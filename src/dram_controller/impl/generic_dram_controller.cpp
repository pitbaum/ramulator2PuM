#include "dram_controller/controller.h"
#include "memory_system/memory_system.h"

namespace Ramulator {

class GenericDRAMController final : public IDRAMController, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAMController, GenericDRAMController, "Generic", "A generic DRAM controller.");
  private:
    std::deque<Request> pending;          // A queue for read requests that are about to finish (callback after RL)

    ReqBuffer m_active_buffer;            // Buffer for requests being served. This has the highest priority 
    ReqBuffer m_priority_buffer;          // Buffer for high-priority requests (e.g., maintenance like refresh).
    ReqBuffer m_read_buffer;              // Read request buffer
    ReqBuffer m_write_buffer;             // Write request buffer
    ReqBuffer m_rc_buffer;                // rowclone request buffer
    ReqBuffer m_maj_buffer;               // majority request buffer
    ReqBuffer m_aggregated_pum;           // once all requests have arrived they are made smaller

    int rc_command_size = 16;   // Amount of rowclone addresses needed to issue a command (2-16)
    int maj_commandsize = 30;   // Amount of majority addresses needed to issue a command (3-30)
    int frac_commands = 2;      // How often the frac should be executed
    int m_bank_addr_idx = -1;

    float m_wr_low_watermark;
    float m_wr_high_watermark;
    bool  m_is_write_mode = false;

    size_t s_row_hits = 0;
    size_t s_row_misses = 0;
    size_t s_row_conflicts = 0;
    size_t s_read_row_hits = 0;
    size_t s_read_row_misses = 0;
    size_t s_read_row_conflicts = 0;
    size_t s_write_row_hits = 0;
    size_t s_write_row_misses = 0;
    size_t s_write_row_conflicts = 0;

    size_t m_num_cores = 0;
    std::vector<size_t> s_read_row_hits_per_core;
    std::vector<size_t> s_read_row_misses_per_core;
    std::vector<size_t> s_read_row_conflicts_per_core;

    size_t s_num_read_reqs = 0;
    size_t s_num_write_reqs = 0;
    size_t s_num_frac_reqs = 0;
    size_t s_num_rc_reqs = 0;
    size_t s_num_maj_reqs = 0;
    size_t s_num_other_reqs = 0;
    size_t s_queue_len = 0;
    size_t s_read_queue_len = 0;
    size_t s_write_queue_len = 0;
    size_t s_rc_queue_len = 0;
    size_t s_maj_queue_len = 0;
    size_t s_frac_queue_len = 0;
    size_t s_priority_queue_len = 0;
    float s_queue_len_avg = 0;
    float s_read_queue_len_avg = 0;
    float s_write_queue_len_avg = 0;
    float s_priority_queue_len_avg = 0;

    size_t s_read_latency = 0;
    float s_avg_read_latency = 0;


  public:
    void init() override {
      m_wr_low_watermark =  param<float>("wr_low_watermark").desc("Threshold for switching back to read mode.").default_val(0.2f);
      m_wr_high_watermark = param<float>("wr_high_watermark").desc("Threshold for switching to write mode.").default_val(0.8f);
      
      // buffer at least 16x32 for max(bank) x max(row addresses)
      m_maj_buffer.max_size = 512;
      m_rc_buffer.max_size = 512;

      m_scheduler = create_child_ifce<IScheduler>();
      m_refresh = create_child_ifce<IRefreshManager>();    
      m_rowpolicy = create_child_ifce<IRowPolicy>();    

      if (m_config["plugins"]) {
        YAML::Node plugin_configs = m_config["plugins"];
        for (YAML::iterator it = plugin_configs.begin(); it != plugin_configs.end(); ++it) {
          m_plugins.push_back(create_child_ifce<IControllerPlugin>(*it));
        }
      }
    };

    /**
     * @brief Checks if there are at least N requests with the same addr_vec in the given buffer.
     *        If so, moves the first such request to the destination buffer and removes the first N such requests from the source buffer.
     *         If it is a maj request that will be pushed to the destination buffer, we will also push 2 fractional commands
     * @param src_buffer The source buffer to search (e.g., m_maj_buffer)
     * @param dst_buffer The destination buffer to move the first matching request (e.g., m_active_buffer)
     * @param N The required number of matching requests
     * @return true if operation was performed, false otherwise
     */
    bool move_n_matching_requests(
      ReqBuffer& src_buffer,
      ReqBuffer& dst_buffer,
      size_t N)
    {
      std::map<std::vector<int>, std::vector<ReqBuffer::iterator>> addr_map;
      for (auto it = src_buffer.begin(); it != src_buffer.end(); ++it) {
          addr_map[it->addr_vec].push_back(it);
      }
      for (auto& [addr_vec, iters] : addr_map) {
        if (iters.size() >= N) {
          // Prepare requests to enqueue
          std::vector<Request> to_enqueue;
          // Add the fractional commands that are necessary if the amount of rows in MAJ is not multiple of 3
          if (iters[0]->type_id == Request::Type::Majority) {
            for (int fractionals = 0; fractionals < frac_commands; fractionals++) {
              to_enqueue.emplace_back(iters[0]->addr, Request::Type::Fractional);
              to_enqueue.back().final_command = 11; // Set the final command manually (FRAC)
              to_enqueue.back().addr_vec = iters[0]->addr_vec; // Set the addressvector manually (should be a prereserved row)
            }
          }
          to_enqueue.push_back(*iters[0]); // enqueue the request
          // Check if there is enough space in the destination buffer
          if (dst_buffer.size() + to_enqueue.size() <= dst_buffer.max_size) {
              // Enqueue all requests
              for (const auto& req : to_enqueue) {
                  dst_buffer.enqueue(req);
              }
              // Remove the first N matching requests from src_buffer
              for (size_t i = 0; i < N; ++i) {
                  src_buffer.remove(iters[i]);
              }
              return true;
          }
          break;
        }
      }
      return false;
    }

    /**
     * @brief Checks all requests in m_aggregated_pum If at least one is ready, set req_it and return true.
     * @param req_it Iterator to the found request (output)
     * @return true if at least one request is not closing a used bank and is ready, false otherwise
     */
    bool check_aggregated_pum_by_bank_and_ready(ReqBuffer::iterator& req_it)
    {
      for (auto pum_it = m_aggregated_pum.begin(); pum_it != m_aggregated_pum.end(); ++pum_it) {
        // this initialize the prerequisit command for all requests in the buffer
        // In the read and write one it is done inside of the get best request function which is hiding this.
        pum_it->command = m_dram->get_preq_command(pum_it->final_command, pum_it->addr_vec);
        // Check if the request is ready and if the prerequisit command is not -1 (state that you cant interrupt with your flow)
        if (pum_it->command != -1 && m_dram->check_ready(pum_it->command, pum_it->addr_vec)) {
          req_it = pum_it;
          return true;
        }
      }
      // All requests are closing used banks or not ready
      return false;
    }

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = memory_system->get_ifce<IDRAM>();
      m_bank_addr_idx = m_dram->m_levels("bank");
      m_priority_buffer.max_size = 512*3 + 32;

      m_num_cores = frontend->get_num_cores();

      s_read_row_hits_per_core.resize(m_num_cores, 0);
      s_read_row_misses_per_core.resize(m_num_cores, 0);
      s_read_row_conflicts_per_core.resize(m_num_cores, 0);

      register_stat(s_row_hits).name("row_hits_{}", m_channel_id);
      register_stat(s_row_misses).name("row_misses_{}", m_channel_id);
      register_stat(s_row_conflicts).name("row_conflicts_{}", m_channel_id);
      register_stat(s_read_row_hits).name("read_row_hits_{}", m_channel_id);
      register_stat(s_read_row_misses).name("read_row_misses_{}", m_channel_id);
      register_stat(s_read_row_conflicts).name("read_row_conflicts_{}", m_channel_id);
      register_stat(s_write_row_hits).name("write_row_hits_{}", m_channel_id);
      register_stat(s_write_row_misses).name("write_row_misses_{}", m_channel_id);
      register_stat(s_write_row_conflicts).name("write_row_conflicts_{}", m_channel_id);

      for (size_t core_id = 0; core_id < m_num_cores; core_id++) {
        register_stat(s_read_row_hits_per_core[core_id]).name("read_row_hits_core_{}", core_id);
        register_stat(s_read_row_misses_per_core[core_id]).name("read_row_misses_core_{}", core_id);
        register_stat(s_read_row_conflicts_per_core[core_id]).name("read_row_conflicts_core_{}", core_id);
      }

      register_stat(s_num_read_reqs).name("num_read_reqs_{}", m_channel_id);
      register_stat(s_num_write_reqs).name("num_write_reqs_{}", m_channel_id);
      register_stat(s_num_other_reqs).name("num_other_reqs_{}", m_channel_id);
      register_stat(s_queue_len).name("queue_len_{}", m_channel_id);
      register_stat(s_read_queue_len).name("read_queue_len_{}", m_channel_id);
      register_stat(s_write_queue_len).name("write_queue_len_{}", m_channel_id);
      register_stat(s_priority_queue_len).name("priority_queue_len_{}", m_channel_id);
      register_stat(s_queue_len_avg).name("queue_len_avg_{}", m_channel_id);
      register_stat(s_read_queue_len_avg).name("read_queue_len_avg_{}", m_channel_id);
      register_stat(s_write_queue_len_avg).name("write_queue_len_avg_{}", m_channel_id);
      register_stat(s_priority_queue_len_avg).name("priority_queue_len_avg_{}", m_channel_id);

      register_stat(s_read_latency).name("read_latency_{}", m_channel_id);
      register_stat(s_avg_read_latency).name("avg_read_latency_{}", m_channel_id);
    };

    bool send(Request& req) override {
      req.final_command = m_dram->m_request_translations(req.type_id);

      switch (req.type_id) {
        case Request::Type::Read: {
          s_num_read_reqs++;
          break;
        }
        case Request::Type::Write: {
          s_num_write_reqs++;
          break;
        }
        case Request::Type::Fractional: { // the Type id is not coherent with internal DDR4 command ids
          s_num_frac_reqs++;
          break;
        }
        case Request::Type::Rowclone: {
          s_num_frac_reqs++;
          break;
        }
        case Request::Type::Majority: {
          s_num_frac_reqs++;
          break;
        }
        default: {
          s_num_other_reqs++;
          break;
        }
      }

      // Forward existing write requests to incoming read requests
      // Reason why some Read requests might not be listed when printing commands, instead of issuing read, just send the write data back
      if (req.type_id == Request::Type::Read) {
        auto compare_addr = [req](const Request& wreq) {
          return wreq.addr == req.addr;
        };
        if (std::find_if(m_write_buffer.begin(), m_write_buffer.end(), compare_addr) != m_write_buffer.end()) {
          // The request will depart at the next cycle
          req.depart = m_clk + 1;
          pending.push_back(req);
          return true;
        }
      }

      // Else, enqueue them to corresponding buffer based on request type id
      bool is_success = false;
      req.arrive = m_clk;
      if (req.type_id == Request::Type::Read) {
        is_success = m_read_buffer.enqueue(req);
      } else if (req.type_id == Request::Type::Write) {
        is_success = m_write_buffer.enqueue(req);
      } else if (req.type_id == Request::Type::Rowclone) {
        is_success = m_rc_buffer.enqueue(req);
      } else if (req.type_id == Request::Type::Majority) {
        is_success = m_maj_buffer.enqueue(req);
      } else if (req.type_id == Request::Type::Fractional) {  // frac no need for several addresses
        is_success = m_aggregated_pum.enqueue(req);
      } else {
        throw std::runtime_error("Invalid request type!");
      }
      if (!is_success) {
        // We could not enqueue the request
        req.arrive = -1;
        return false;
      }

      return true;
    };

    bool priority_send(Request& req) override {
      req.final_command = m_dram->m_request_translations(req.type_id);

      bool is_success = false;
      is_success = m_priority_buffer.enqueue(req);
      return is_success;
    }

    void tick() override {
      m_clk++;

      // Update statistics
      s_queue_len += m_read_buffer.size() + m_write_buffer.size() + m_priority_buffer.size() + pending.size();
      s_read_queue_len += m_read_buffer.size() + pending.size();
      s_write_queue_len += m_write_buffer.size();
      s_priority_queue_len += m_priority_buffer.size();

      // 1. Serve completed reads
      serve_completed_reads();

      m_refresh->tick();

      // 2. Try to find a request to serve.
      ReqBuffer::iterator req_it;
      ReqBuffer* buffer = nullptr;
      bool request_found = schedule_request(req_it, buffer);

      // 2.1 Take row policy action
      m_rowpolicy->update(request_found, req_it);

      // 3. Update all plugins
      for (auto plugin : m_plugins) {
        plugin->update(request_found, req_it);
      }

      // 4. Finally, issue the commands to serve the request
      if (request_found) {
        // If we find a real request to serve
        if (req_it->is_stat_updated == false) {
          update_request_stats(req_it);
        }
        m_dram->issue_command(req_it->command, req_it->addr_vec);

        // If we are issuing the last command, set depart clock cycle and move the request to the pending queue
        if (req_it->command == req_it->final_command) {
          if (req_it->type_id == Request::Type::Read) {
            req_it->depart = m_clk + m_dram->m_read_latency;
            pending.push_back(*req_it);
          } else if (req_it->type_id == Request::Type::Write) {
            // TODO: Add code to update statistics
          }
          buffer->remove(req_it);
        } else {
          if (m_dram->m_command_meta(req_it->command).is_opening) {
            if (m_active_buffer.enqueue(*req_it)) {
              buffer->remove(req_it);
            }
          }
        }

      }

    };


  private:
    /**
     * @brief    Helper function to check if a request is hitting an open row
     * @details
     * 
     */
    bool is_row_hit(ReqBuffer::iterator& req)
    {
        return m_dram->check_rowbuffer_hit(req->final_command, req->addr_vec);
    }
    /**
     * @brief    Helper function to check if a request is opening a row
     * @details
     * 
    */
    bool is_row_open(ReqBuffer::iterator& req)
    {
        return m_dram->check_node_open(req->final_command, req->addr_vec);
    }

    /**
     * @brief    
     * @details
     * 
     */
    void update_request_stats(ReqBuffer::iterator& req)
    {
      req->is_stat_updated = true;

      if (req->type_id == Request::Type::Read) 
      {
        if (is_row_hit(req)) {
          s_read_row_hits++;
          s_row_hits++;
          if (req->source_id != -1)
            s_read_row_hits_per_core[req->source_id]++;
        } else if (is_row_open(req)) {
          s_read_row_conflicts++;
          s_row_conflicts++;
          if (req->source_id != -1)
            s_read_row_conflicts_per_core[req->source_id]++;
        } else {
          s_read_row_misses++;
          s_row_misses++;
          if (req->source_id != -1)
            s_read_row_misses_per_core[req->source_id]++;
        } 
      } 
      else if (req->type_id == Request::Type::Write) 
      {
        if (is_row_hit(req)) {
          s_write_row_hits++;
          s_row_hits++;
        } else if (is_row_open(req)) {
          s_write_row_conflicts++;
          s_row_conflicts++;
        } else {
          s_write_row_misses++;
          s_row_misses++;
        }
      }
    }

    /**
     * @brief    Helper function to serve the completed read requests
     * @details
     * This function is called at the beginning of the tick() function.
     * It checks the pending queue to see if the top request has received data from DRAM.
     * If so, it finishes this request by calling its callback and poping it from the pending queue.
     */
    void serve_completed_reads() {
      if (pending.size()) {
        // Check the first pending request
        auto& req = pending[0];
        if (req.depart <= m_clk) {
          // Request received data from dram
          if (req.depart - req.arrive > 1) {
            // Check if this requests accesses the DRAM or is being forwarded.
            // TODO add the stats back
            s_read_latency += req.depart - req.arrive;
          }

          if (req.callback) {
            // If the request comes from outside (e.g., processor), call its callback
            req.callback(req);
          }
          // Finally, remove this request from the pending queue
          pending.pop_front();
        }
      };
    };


    /**
     * @brief    Checks if we need to switch to write mode
     * 
     */
    void set_write_mode() {
      if (!m_is_write_mode) {
        if ((m_write_buffer.size() > m_wr_high_watermark * m_write_buffer.max_size) || m_read_buffer.size() == 0) {
          m_is_write_mode = true;
        }
      } else {
        if ((m_write_buffer.size() < m_wr_low_watermark * m_write_buffer.max_size) && m_read_buffer.size() != 0) {
          m_is_write_mode = false;
        }
      }
    };


    /**
     * @brief    Helper function to find a request to schedule from the buffers.
     * 
     *     Active requests > Priority requests > PuM requests > Read & Write Requests
     */
    bool schedule_request(ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer) {
      bool request_found = false;
      bool is_active_buffer = false;
      // 2.1    First, check the act buffer to serve requests that are already activating (avoid useless ACTs)
      if (req_it= m_scheduler->get_best_request(m_active_buffer); req_it != m_active_buffer.end()) {
        if (m_dram->check_ready(req_it->command, req_it->addr_vec)) {
          request_found = true;
          req_buffer = &m_active_buffer;
          return request_found;
        }
      }

      // 2.2    If no requests can be scheduled from the act buffer or not issuable, check the rest of the buffers
      if (!request_found || req_it->command == -1) {
        // 2.2.1    We first check the priority buffer to prioritize e.g., maintenance requests
        if (m_priority_buffer.size() != 0) {
          req_buffer = &m_priority_buffer;
          req_it = m_priority_buffer.begin();
          req_it->command = m_dram->get_preq_command(req_it->final_command, req_it->addr_vec);
          // Only check if a command is ready if their prerequisit is not -1
          // -1 Being that you are in a not interruptable state (unfinished PuM state, RDWR open instead of PuM open)
          if (req_it->command != -1){
            request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
          }
          if (!request_found & m_priority_buffer.size() != 0) {
            return false;
          }
        }

        // 2.2.1 If no request to be scheduled in the priority buffer or not issuable at this state, check the PuM buffers.
        // Check for the PuM buffer size, if it is big enough for an issue. Check the address is for same request.
        // If we find pum commands requests that have arrived fully, we move them to aggregrate pum buffer for further check
        if (m_aggregated_pum.size() + 1 <= m_aggregated_pum.max_size) {
          move_n_matching_requests(m_maj_buffer, m_aggregated_pum, maj_commandsize);
          move_n_matching_requests(m_rc_buffer, m_aggregated_pum, rc_command_size);
        }

        if(!request_found || req_it->command == -1) {
          // If we have a fully arrived and aggregated pum command at least in the buffer, check if it is issuable
          if(m_aggregated_pum.size() > 0) {
            request_found = check_aggregated_pum_by_bank_and_ready(req_it);
            if (request_found) {
              req_buffer = &m_aggregated_pum;
            }
          }
        }

        // 2.2.1 If no request to be scheduled in the PuM buffer or request invalid, check the read and write buffers.
        if (!request_found || req_it->command == -1) {
          // Query the write policy to decide which buffer to serve
          set_write_mode();
          auto& buffer = m_is_write_mode ? m_write_buffer : m_read_buffer;
          if (req_it = m_scheduler->get_best_request(buffer); req_it != buffer.end()) {
            request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
            req_buffer = &buffer;
          }
        }
      }

      // If the prerequisit to the command you found is -1, the command is actually not really ready
      // The scheduler should always prioritize requests that are not -1
      // Thus if we still get one here, we just return request was not found
      if (req_it->command == -1) {
        request_found = false;
        return request_found;
      }

      // 2.3 If we find a request to schedule, we need to check if it will close an opened row in the active buffer.
      if (request_found) {
        if (m_dram->m_command_meta(req_it->command).is_closing) {
          auto& rowgroup = req_it->addr_vec;
          for (auto _it = m_active_buffer.begin(); _it != m_active_buffer.end(); _it++) {
            auto& _it_rowgroup = _it->addr_vec;
            bool is_matching = true;
            for (int i = 0; i < m_bank_addr_idx + 1 ; i++) {
              if (_it_rowgroup[i] != rowgroup[i] && _it_rowgroup[i] != -1 && rowgroup[i] != -1) {
                is_matching = false;
                break;
              }
            }
            if (is_matching) {
              request_found = false;
              break;
            }
          }
        }
      }

      // 2.4 If we find a request to schedule, we need to check if it will interrupt an already active PuM command given inter bank and bankgroup switching times.
      // If the best request we found is not fast enough to not issue and return when the APA command is scheduled in active buffer, then dont issue the found request.
      // Additionally make sure that no Request is issued that would interfere with a tightly constraint command in the APA chain
      // Tightly constraint PuM commands are all the APA commands, though the only one with realistic switching time to schedule inbetween is the RC ACTp -> PREv
      if (request_found) {
        int channel_index = 0; // in ChRaBgBaRoCo organization
        int rank_index = 1;
        int bankgroup_index = 2;
        int ACTv_index = 13; // in DDR4 standard
        int PREj_index = 15;
        int PREv_index = 14;
        int ACTp_index = 12; // PREf not necessary, if that would have been the case it would have already been issued
        auto& rowgroup = req_it->addr_vec;
        for (auto _it = m_active_buffer.begin(); _it != m_active_buffer.end(); _it++) {
          // Only enforce if the active buffer command scheduled is PuM related
          if (_it->command == ACTv_index || _it->command == PREj_index || _it->command == PREv_index || _it->command == ACTp_index) {
            auto& _it_rowgroup = _it->addr_vec;
            // Only necessary if we are on the same DIMM (i.e. same rank, same channel)
            // Rank has full parallelism in terms of ACT and PRE commands so we dont need to add additional constraint to it
            for (int i = 0; i < m_bank_addr_idx + 1 ; i++) {
              if (_it_rowgroup[channel_index] == rowgroup[channel_index] && _it_rowgroup[channel_index] == rowgroup[channel_index] && _it_rowgroup[i] != -1 && rowgroup[i] != -1) {
                // Same bankgroup as something in active buffer
                bool is_same_bg = _it_rowgroup[bankgroup_index] == rowgroup[bankgroup_index];
                request_found = m_dram->check_interuption_with_delay(_it->command, _it->final_command, req_it->command, req_it->final_command, _it->addr_vec, req_it->addr_vec, true);
              }
            }
          }
        }
      }
      return request_found;
    }

    void finalize() override {
      s_avg_read_latency = (float) s_read_latency / (float) s_num_read_reqs;

      s_queue_len_avg = (float) s_queue_len / (float) m_clk;
      s_read_queue_len_avg = (float) s_read_queue_len / (float) m_clk;
      s_write_queue_len_avg = (float) s_write_queue_len / (float) m_clk;
      s_priority_queue_len_avg = (float) s_priority_queue_len / (float) m_clk;

      return;
    }

};
  
}   // namespace Ramulator
