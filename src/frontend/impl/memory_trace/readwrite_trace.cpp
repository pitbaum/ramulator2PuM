#include <filesystem>
#include <iostream>
#include <fstream>

#include "frontend/frontend.h"
#include "base/exception.h"

namespace Ramulator {

namespace fs = std::filesystem;

class ReadWriteTrace : public IFrontEnd, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IFrontEnd, ReadWriteTrace, "ReadWriteTrace", "Read/Write DRAM address vector trace.")

  private:
    struct Trace {
      int request_type_enum;
      Addr_t addr;
    };
    std::vector<Trace> m_trace;

    size_t m_trace_length = 0;
    size_t m_curr_trace_idx = 0;

    Logger_t m_logger;

  public:
    void init() override {
      std::string trace_path_str = param<std::string>("path").desc("Path to the load store trace file.").required();
      m_clock_ratio = param<uint>("clock_ratio").required();

      m_logger = Logging::create_logger("ReadWriteTrace");
      m_logger->info("Loading trace file {} ...", trace_path_str);
      init_trace(trace_path_str);
      m_logger->info("Loaded {} lines.", m_trace.size());      
    };


    void tick() override {
      const Trace& t = m_trace[m_curr_trace_idx];
      // Here we send the request to the front end
      // Needed to change binary w ? r to the new enum in the request
      m_memory_system->send({t.addr, t.request_type_enum});
      m_curr_trace_idx = (m_curr_trace_idx + 1); // They were using % m_trace_length????? that would make infinite loop
    };


  private:
    void init_trace(const std::string& file_path_str) {
      fs::path trace_path(file_path_str);
      if (!fs::exists(trace_path)) {
        throw ConfigurationError("Trace {} does not exist!", file_path_str);
      }

      std::ifstream trace_file(trace_path);
      if (!trace_file.is_open()) {
        throw ConfigurationError("Trace {} cannot be opened!", file_path_str);
      }

      std::string line;
      while (std::getline(trace_file, line)) {
        std::vector<std::string> tokens;
        tokenize(tokens, line, " ");

        // TODO: Add line number here for better error messages
        if (tokens.size() != 2) {
          throw ConfigurationError("Trace {} format invalid!", file_path_str);
        }

        // This is where the trace evaluation  is done        bool is_write = false; 
        int request_type_enum = 0; // Needs to match our request command type enum 
        switch (tokens[0][0]) {
          case 'R':
            request_type_enum = 0;
            break;
          case 'W':
            request_type_enum = 1;
            break;
          case 'F':
            request_type_enum = 7; // Those are the indexes of the LUT for DDR4 m_commands
            break;
          case 'C':
            request_type_enum = 5;
            break;
          case 'M':
            request_type_enum = 6;
            break;
          default:
            throw ConfigurationError("Trace {} format invalid!", file_path_str);
        }
        Addr_t addr = std::stoll(tokens[1]);
        m_trace.push_back({request_type_enum, addr});
      }

      trace_file.close();

      m_trace_length = m_trace.size();
    };

    bool is_finished() override {
      // If our send request traces are as >= the amount of traces we send, the frontend is finished
      return m_curr_trace_idx >= m_trace_length;
    };    
};

}        // namespace Ramulator