#include "dram/dram.h"
#include "dram/lambdas.h"

namespace Ramulator {

class DDR4 : public IDRAM, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAM, DDR4, "DDR4", "DDR4 Device Model")

  public:
    inline static const std::map<std::string, Organization> org_presets = {
      //   name         density   DQ   Ch Ra Bg Ba   Ro     Co
      {"DDR4_2Gb_x4",   {2<<10,   4,  {1, 1, 4, 4, 1<<15, 1<<10}}},
      {"DDR4_2Gb_x8",   {2<<10,   8,  {1, 1, 4, 4, 1<<14, 1<<10}}},
      {"DDR4_2Gb_x16",  {2<<10,   16, {1, 1, 2, 4, 1<<14, 1<<10}}},
      {"DDR4_4Gb_x4",   {4<<10,   4,  {1, 1, 4, 4, 1<<16, 1<<10}}},
      {"DDR4_4Gb_x8",   {4<<10,   8,  {1, 1, 4, 4, 1<<15, 1<<10}}},
      {"DDR4_4Gb_x16",  {4<<10,   16, {1, 1, 2, 4, 1<<15, 1<<10}}},
      {"DDR4_8Gb_x4",   {8<<10,   4,  {1, 1, 4, 4, 1<<17, 1<<10}}},
      {"DDR4_8Gb_x8",   {8<<10,   8,  {1, 1, 4, 4, 1<<16, 1<<10}}},
      {"DDR4_8Gb_x16",  {8<<10,   16, {1, 1, 2, 4, 1<<16, 1<<10}}},
      {"DDR4_16Gb_x4",  {16<<10,  4,  {1, 1, 4, 4, 1<<18, 1<<10}}},
      {"DDR4_16Gb_x8",  {16<<10,  8,  {1, 1, 4, 4, 1<<17, 1<<10}}},
      {"DDR4_16Gb_x16", {16<<10,  16, {1, 1, 2, 4, 1<<17, 1<<10}}},
    };

    inline static const std::map<std::string, std::vector<int>> timing_presets = {
      //   name       rate   nBL  nCL  nRCD  nRP   nRAS  nRC   nWR  nRTP nCWL nCCDS nCCDL nRRDS nRRDL nWTRS nWTRL nFAW  nRFC nREFI nCS,  tCK_ps
      {"DDR4_1600J",  {1600,   4,  10,  10,   10,   28,   38,   12,   6,   9,    4,    5,   -1,   -1,    2,    6,   -1,  -1,  -1,   2,    1250}},
      {"DDR4_1600K",  {1600,   4,  11,  11,   11,   28,   39,   12,   6,   9,    4,    5,   -1,   -1,    2,    6,   -1,  -1,  -1,   2,    1250}},
      {"DDR4_1600L",  {1600,   4,  12,  12,   12,   28,   40,   12,   6,   9,    4,    5,   -1,   -1,    2,    6,   -1,  -1,  -1,   2,    1250}},
      {"DDR4_1866L",  {1866,   4,  12,  12,   12,   32,   44,   14,   7,   10,   4,    5,   -1,   -1,    3,    7,   -1,  -1,  -1,   2,    1071}},
      {"DDR4_1866M",  {1866,   4,  13,  13,   13,   32,   45,   14,   7,   10,   4,    5,   -1,   -1,    3,    7,   -1,  -1,  -1,   2,    1071}},
      {"DDR4_1866N",  {1866,   4,  14,  14,   14,   32,   46,   14,   7,   10,   4,    5,   -1,   -1,    3,    7,   -1,  -1,  -1,   2,    1071}},
      {"DDR4_2133N",  {2133,   4,  14,  14,   14,   36,   50,   16,   8,   11,   4,    6,   -1,   -1,    3,    8,   -1,  -1,  -1,   2,    937} },
      {"DDR4_2133P",  {2133,   4,  15,  15,   15,   36,   51,   16,   8,   11,   4,    6,   -1,   -1,    3,    8,   -1,  -1,  -1,   2,    937} },
      {"DDR4_2133R",  {2133,   4,  16,  16,   16,   36,   52,   16,   8,   11,   4,    6,   -1,   -1,    3,    8,   -1,  -1,  -1,   2,    937} },
      {"DDR4_2400P",  {2400,   4,  15,  15,   15,   39,   54,   18,   9,   12,   4,    6,   -1,   -1,    3,    9,   -1,  -1,  -1,   2,    833} },
      {"DDR4_2400R",  {2400,   4,  16,  16,   16,   39,   55,   18,   9,   12,   4,    6,   -1,   -1,    3,    9,   -1,  -1,  -1,   2,    833} },
      {"DDR4_2400U",  {2400,   4,  17,  17,   17,   39,   56,   18,   9,   12,   4,    6,   -1,   -1,    3,    9,   -1,  -1,  -1,   2,    833} },
      {"DDR4_2400T",  {2400,   4,  18,  18,   18,   39,   57,   18,   9,   12,   4,    6,   -1,   -1,    3,    9,   -1,  -1,  -1,   2,    833} },
      {"DDR4_2666T",  {2666,   4,  17,  17,   17,   43,   60,   20,   10,  14,   4,    7,   -1,   -1,    4,    10,  -1,  -1,  -1,   2,    750} },
      {"DDR4_2666U",  {2666,   4,  18,  18,   18,   43,   61,   20,   10,  14,   4,    7,   -1,   -1,    4,    10,  -1,  -1,  -1,   2,    750} },
      {"DDR4_2666V",  {2666,   4,  19,  19,   19,   43,   62,   20,   10,  14,   4,    7,   -1,   -1,    4,    10,  -1,  -1,  -1,   2,    750} },
      {"DDR4_2666W",  {2666,   4,  20,  20,   20,   43,   63,   20,   10,  14,   4,    7,   -1,   -1,    4,    10,  -1,  -1,  -1,   2,    750} },
      {"DDR4_2933V",  {2933,   4,  19,  19,   19,   47,   66,   22,   11,  16,   4,    8,   -1,   -1,    4,    11,  -1,  -1,  -1,   2,    682} },
      {"DDR4_2933W",  {2933,   4,  20,  20,   20,   47,   67,   22,   11,  16,   4,    8,   -1,   -1,    4,    11,  -1,  -1,  -1,   2,    682} },
      {"DDR4_2933Y",  {2933,   4,  21,  21,   21,   47,   68,   22,   11,  16,   4,    8,   -1,   -1,    4,    11,  -1,  -1,  -1,   2,    682} },
      {"DDR4_2933AA", {2933,   4,  22,  22,   22,   47,   69,   22,   11,  16,   4,    8,   -1,   -1,    4,    11,  -1,  -1,  -1,   2,    682} },
      {"DDR4_3200W",  {3200,   4,  20,  20,   20,   52,   72,   24,   12,  16,   4,    8,   -1,   -1,    4,    12,  -1,  -1,  -1,   2,    625} },
      {"DDR4_3200AA", {3200,   4,  22,  22,   22,   52,   74,   24,   12,  16,   4,    8,   -1,   -1,    4,    12,  -1,  -1,  -1,   2,    625} },
      {"DDR4_3200AC", {3200,   4,  24,  24,   24,   52,   76,   24,   12,  16,   4,    8,   -1,   -1,    4,    12,  -1,  -1,  -1,   2,    625} },
       //   name       rate   nBL  nCL  nRCD  nRP   nRAS  nRC   nWR  nRTP nCWL nCCDS nCCDL nRRDS nRRDL nWTRS nWTRL nFAW  nRFC nREFI nCS,  tCK_ps
      {"DDR4_4000",   {4000,   4,  16,  21,   13,   36,   60,   16,   10,  16,   4,    8,   -1,   -1,    3,    6,    -1,  -1,  -1,   2,   500} },
    };

    inline static const std::map<std::string, std::vector<double>> voltage_presets = {
      //   name          VDD      VPP
      {"Default",       {1.2,     2.5}},
    };

    inline static const std::map<std::string, std::vector<double>> current_presets = {
      // name           IDD0  IDD2N   IDD3N   IDD4R   IDD4W   IDD5B   IPP0  IPP2N  IPP3N  IPP4R  IPP4W  IPP5B
      {"Default",       {60,   50,     55,     145,    145,    362,     3,    3,     3,     3,     3,     48}},
    };

  /************************************************
   *                Organization
   ***********************************************/   
    const int m_internal_prefetch_size = 8;

    inline static constexpr ImplDef m_levels = {
      "channel", "rank", "bankgroup", "bank", "row", "column",    
    };


  /************************************************
   *             Requests & Commands
   ***********************************************/
    inline static constexpr ImplDef m_commands = {
      "ACT", 
      "PRE", "PREA",
      "RD",  "WR",  "RDA",  "WRA",
      "REFab", "REFab_end",
      "RC", "MAJ", "FRAC", "ACTp", "ACTv", "PREv", "PREj", "PREf",
    };

    inline static const ImplLUT m_command_scopes = LUT (
      m_commands, m_levels, {
        {"ACT",   "row"},
        {"PRE",   "bank"},   {"PREA",   "rank"},
        {"RD",    "column"}, {"WR",     "column"}, {"RDA",   "column"}, {"WRA",   "column"},
        {"REFab", "rank"},  {"REFab_end", "rank"},
        {"RC",    "bank"}, {"MAJ", "bank"}, {"FRAC", "bank"},
        {"ACTp", "row"}, {"ACTv",  "row"}, {"PREv", "bank"}, {"PREj", "bank"}, {"PREf", "bank"}, 
      }
    );

    inline static const ImplLUT m_command_meta = LUT<DRAMCommandMeta> (
      m_commands, {
                    // open?   close?   access?  refresh?
        {"ACT",       {true,   false,   false,   false}},
        {"PRE",       {false,  true,    false,   false}},
        {"PREA",      {false,  true,    false,   false}},
        {"RD",        {false,  false,   true,    false}},
        {"WR",        {false,  false,   true,    false}},
        {"RDA",       {false,  true,    true,    false}},
        {"WRA",       {false,  true,    true,    false}},
        {"REFab",     {false,  false,   false,   true }},
        {"REFab_end", {false,  true,    false,   false}},
        // Put the initial command ACTp to openting such that it will move the request to active buffer
        // Put all the PuM commands to closing since this is only for the scheduler
        // We want there to be not interuptions to active buffer hits after starting a PuM with ACTp
        {"RC",        {false,  true,    false,   false}}, 
        {"MAJ",       {false,  true,    false,   false}},
        {"FRAC",      {false,  true,    false,   false}},
        {"ACTp",      {true,   false,   false,   false}},
        {"ACTv",      {false,  true,   false,   false}},
        {"PREv",      {false,  true,   false,   false}},
        {"PREj",      {false,  true,   false,   false}},
        {"PREf",      {false,  true,   false,   false}},
      }
    );

    inline static constexpr ImplDef m_requests = {
      "read", "write", "all-bank-refresh", "open-row", "close-row", "rowclone", "majority", "fractional"
    };

    inline static const ImplLUT m_request_translations = LUT (
      m_requests, m_commands, {
        {"read", "RD"}, {"write", "WR"}, {"all-bank-refresh", "REFab"},
        {"open-row", "ACT"}, {"close-row", "PRE"}, {"rowclone", "RC"}, 
        {"majority", "MAJ"}, {"fractional", "FRAC"}
      }
    );

   
  /************************************************
   *                   Timing
   ***********************************************/
    inline static constexpr ImplDef m_timings = {
      "rate", 
      "nBL", "nCL", "nRCD", "nRP", "nRAS", "nRC", "nWR", "nRTP", "nCWL",
      "nCCDS", "nCCDL",
      "nRRDS", "nRRDL",
      "nWTRS", "nWTRL",
      "nFAW",
      "nRFC","nREFI",
      "nCS",
      "tCK_ps"
    };

   
  /************************************************
   *                   Power
   ***********************************************/
    inline static constexpr ImplDef m_voltages = {
      "VDD", "VPP"
    };
    
    inline static constexpr ImplDef m_currents = {
      "IDD0", "IDD2N", "IDD3N", "IDD4R", "IDD4W", "IDD5B",
      "IPP0", "IPP2N", "IPP3N", "IPP4R", "IPP4W", "IPP5B"
    };

    inline static constexpr ImplDef m_cmds_counted = {
      "ACT", "PRE", "RD", "WR", "REF"
    };

  /************************************************
   *                 Node States
   ***********************************************/
    inline static constexpr ImplDef m_states = {
       "Opened", "Closed", "PowerUp", "N/A", "Refreshing", "OpenedPum", "RCState", "MAJState", "Processed"
    };

    inline static const ImplLUT m_init_states = LUT (
      m_levels, m_states, {
        {"channel",   "N/A"}, 
        {"rank",      "PowerUp"},
        {"bankgroup", "N/A"},
        {"bank",      "Closed"},
        {"row",       "Closed"},
        {"column",    "N/A"}
      }
    );

  public:
    struct Node : public DRAMNodeBase<DDR4> {
      Node(DDR4* dram, Node* parent, int level, int id) : DRAMNodeBase<DDR4>(dram, parent, level, id) {};
    };
    std::vector<Node*> m_channels;
    
    FuncMatrix<ActionFunc_t<Node>>  m_actions;
    FuncMatrix<PreqFunc_t<Node>>    m_preqs;
    FuncMatrix<RowhitFunc_t<Node>>  m_rowhits;
    FuncMatrix<RowopenFunc_t<Node>> m_rowopens;
    FuncMatrix<PowerFunc_t<Node>>   m_powers;

  public:
    void tick() override {
      m_clk++;
      
      // Check if there is any future action at this cycle
      for (int i = m_future_actions.size() - 1; i >= 0; i--) {
        auto& future_action = m_future_actions[i];
        if (future_action.clk == m_clk) {
          handle_future_action(future_action.cmd, future_action.addr_vec);
          m_future_actions.erase(m_future_actions.begin() + i);
        }
      }
    };

    void init() override {
      RAMULATOR_DECLARE_SPECS();
      set_organization();
      set_timing_vals();

      set_actions();
      set_preqs();
      set_rowhits();
      set_rowopens();
      set_powers();
      
      create_nodes();
    };

    void issue_command(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      m_channels[channel_id]->update_timing(command, addr_vec, m_clk);
      m_channels[channel_id]->update_powers(command, addr_vec, m_clk);
      m_channels[channel_id]->update_states(command, addr_vec, m_clk);
      
      static const std::vector<std::string> DDR4CommandNames = {
        "ACT", 
        "PRE", "PREA",
        "RD",  "WR",  "RDA",  "WRA",
        "REFab", "REFab_end",
        "RC", "MAJ", "FRAC", "ACTp", "ACTv", "PREv", "PREj", "PREf"
      };
      // Print commands issued and when
      std::cout << "Command: " << DDR4CommandNames[command]
            << ", AddrVec: ";
      for (const auto& v : addr_vec) {
        std::cout << v << " ";
      }
      std::cout << ", m_clk: " << m_clk << std::endl;    

      // Check if the command requires future action
      check_future_action(command, addr_vec);
    };

    void check_future_action(int command, const AddrVec_t& addr_vec) {
      switch (command) {
        case m_commands("REFab"):
          // REFab command requires future action after nRFC cycles
          m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFC") - 1});
          break;
        default:
          // Other commands do not require future actions
          break;
      }
    }

    void handle_future_action(int command, const AddrVec_t& addr_vec) {
      int channel_id = addr_vec[m_levels["channel"]];
      switch (command) {
        case m_commands("REFab"):
          m_channels[channel_id]->update_powers(m_commands("REFab_end"), addr_vec, m_clk);
          m_channels[channel_id]->update_states(m_commands("REFab_end"), addr_vec, m_clk);
          break;
        default:
          // Other commands do not require future actions
          break;
      }
    };

    int get_preq_command(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->get_preq_command(command, addr_vec, m_clk);
    };

    bool check_ready(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_ready(command, addr_vec, m_clk);
    };

    // Check if you can issue some command without interupting some PuM APA command schedules in a way that would violate their tight scheduling
    bool check_interuption_with_delay(int active_command, int final_command, int found_command, int found_final_command, const AddrVec_t& active_addr_vec, const AddrVec_t& found_addr_vec, bool is_bg){
      // Set which delay to check on
      int issuing_delay = 4;
      if (is_bg) {
        issuing_delay = 8; // from org
      }
      // Get the clk cycle of when the tightly constraint command will be scheduled in this channel
      int active_ready_clock_cylce = m_channels[active_addr_vec[m_levels["channel"]]]->get_ready_clk_active(active_command, active_addr_vec, m_clk);
      int found_ready_clock_cylce = m_channels[found_addr_vec[m_levels["channel"]]]->get_ready_clk_active(found_command, found_addr_vec, m_clk);

      // Set command names (should cleanup and use from the enum in ddr4 standard above)
      int rc = 9, prev = 14, actp = 12, frac = 11, maj = 10, prej = 15, actv = 13;

      // The only one that can realistacially have enough time to schedule something inbetween is the RC ACTp -> PREv command delay (36 clk)
      // All others the condition to say if you can switch back and forth and still issue, satisfies the rules due to the too small delay between commands that wont fit anything inbetween anyways
      if (final_command == rc && active_command == prev) {
        // MAJ inbetween
        if (found_final_command == maj) {
          if (found_command == actp) {
            // MAJ operation needs to be fully issuable before scheduling it
            return (active_ready_clock_cylce > found_ready_clock_cylce + 9 + issuing_delay);
          }
            // Technically this is implicit that PREj and ACTv have enough time to issue
          if (found_command == prej) {
            return (active_ready_clock_cylce > found_ready_clock_cylce + 6 + issuing_delay);
          } 
          if (found_command == actv) {
            return (active_ready_clock_cylce > found_ready_clock_cylce + issuing_delay);
          }
        }
        // FRAC inbetween
        if (found_final_command == frac) {
          // Issue as long as the next cycle is also free
          if (found_command == actp) {
            return (active_ready_clock_cylce > found_ready_clock_cylce + 1 + issuing_delay);
          }
          // Issue as long as the next cycle is also free
          if (found_command == actp) {
            return (active_ready_clock_cylce > found_ready_clock_cylce + issuing_delay);
          }
        }
        // RC inbetween
        if (found_final_command == rc) {
          // issue as long you can switch back in time  for the active PREv
          // AND 
          // The issuing is at least PREv->ACTv from ACTp active away (6 cylces), to make sure the PREv -> ACTv time windows dont overlap for them
          if (found_command == actp) {
            return (active_ready_clock_cylce > found_ready_clock_cylce + issuing_delay && active_ready_clock_cylce > 6);
          }
        }
      }
      
      // Anything else is not tight scheduled, issue as long as you can switch back in time to issue the tight active command
      return (active_ready_clock_cylce > found_ready_clock_cylce + issuing_delay);
      
    };

    bool check_rowbuffer_hit(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_rowbuffer_hit(command, addr_vec, m_clk);
    };
    
    bool check_node_open(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_node_open(command, addr_vec, m_clk);
    };

  private:
    void set_organization() {
      // Channel width
      m_channel_width = param_group("org").param<int>("channel_width").default_val(64);

      // Organization
      m_organization.count.resize(m_levels.size(), -1);

      // Load organization preset if provided
      if (auto preset_name = param_group("org").param<std::string>("preset").optional()) {
        if (org_presets.count(*preset_name) > 0) {
          m_organization = org_presets.at(*preset_name);
        } else {
          throw ConfigurationError("Unrecognized organization preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      // Override the preset with any provided settings
      if (auto dq = param_group("org").param<int>("dq").optional()) {
        m_organization.dq = *dq;
      }

      for (int i = 0; i < m_levels.size(); i++){
        auto level_name = m_levels(i);
        if (auto sz = param_group("org").param<int>(level_name).optional()) {
          m_organization.count[i] = *sz;
        }
      }

      if (auto density = param_group("org").param<int>("density").optional()) {
        m_organization.density = *density;
      }

      // Sanity check: is the calculated chip density the same as the provided one?
      size_t _density = size_t(m_organization.count[m_levels["bankgroup"]]) *
                        size_t(m_organization.count[m_levels["bank"]]) *
                        size_t(m_organization.count[m_levels["row"]]) *
                        size_t(m_organization.count[m_levels["column"]]) *
                        size_t(m_organization.dq);
      _density >>= 20;
      if (m_organization.density != _density) {
        throw ConfigurationError(
            "Calculated {} chip density {} Mb does not equal the provided density {} Mb!", 
            get_name(),
            _density, 
            m_organization.density
        );
      }
    };

    void set_timing_vals() {
      m_timing_vals.resize(m_timings.size(), -1);

      // Load timing preset if provided
      bool preset_provided = false;
      if (auto preset_name = param_group("timing").param<std::string>("preset").optional()) {
        if (timing_presets.count(*preset_name) > 0) {
          m_timing_vals = timing_presets.at(*preset_name);
          preset_provided = true;
        } else {
          throw ConfigurationError("Unrecognized timing preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      // Check for rate (in MT/s), and if provided, calculate and set tCK (in picosecond)
      if (auto dq = param_group("timing").param<int>("rate").optional()) {
        if (preset_provided) {
          throw ConfigurationError("Cannot change the transfer rate of {} when using a speed preset !", get_name());
        }
        m_timing_vals("rate") = *dq;
      }
      int tCK_ps = 1E6 / (m_timing_vals("rate") / 2);
      m_timing_vals("tCK_ps") = tCK_ps;

      // Load the organization specific timings
      int dq_id = [](int dq) -> int {
        switch (dq) {
          case 4:  return 0;
          case 8:  return 1;
          case 16: return 2;
          default: return -1;
        }
      }(m_organization.dq);

      int rate_id = [](int rate) -> int {
        switch (rate) {
          case 1600:  return 0;
          case 1866:  return 1;
          case 2133:  return 2;
          case 2400:  return 3;
          case 2666:  return 4;
          case 2933:  return 5;
          case 3200:  return 6;
          case 4000:  return 7;
          default:    return -1;
        }
      }(m_timing_vals("rate"));

      // Tables for secondary timings determined by the frequency, density, and DQ width.
      // Defined in the JEDEC standard (e.g., Table 169-170, JESD79-4C).
      constexpr int nRRDS_TABLE[3][8] = {
      // 1600  1866  2133  2400  2666  2933  3200  4000
        { 4,    4,    4,    4,    4,    4,    4,    4},   // x4
        { 4,    4,    4,    4,    4,    4,    4,    4},   // x8
        { 5,    5,    6,    7,    8,    8,    9,    9},   // x16
      };
      constexpr int nRRDL_TABLE[3][8] = {
      // 1600  1866  2133  2400  2666  2933  3200  4000
        { 5,    5,    6,    6,    7,    8,    8,     8},  // x4
        { 5,    5,    6,    6,    7,    8,    8,     8},  // x8
        { 6,    6,    7,    8,    9,    10,   11,   11},  // x16
      };
      constexpr int nFAW_TABLE[3][8] = {
      // 1600  1866  2133  2400  2666  2933  3200   4000
        { 16,   16,   16,   16,   16,   16,   16,   16},  // x4
        { 20,   22,   23,   26,   28,   31,   34,   34},  // x8
        { 28,   28,   32,   36,   40,   44,   48,   48},  // x16
      };

      if (dq_id != -1 && rate_id != -1) {
        m_timing_vals("nRRDS") = nRRDS_TABLE[dq_id][rate_id];
        m_timing_vals("nRRDL") = nRRDL_TABLE[dq_id][rate_id];
        m_timing_vals("nFAW")  = nFAW_TABLE [dq_id][rate_id];
      }

      // Refresh timings
      // tRFC table (unit is nanosecond!)
      constexpr int tRFC_TABLE[3][4] = {
      //  2Gb   4Gb   8Gb  16Gb
        { 160,  260,  360,  550}, // Normal refresh (tRFC1)
        { 110,  160,  260,  350}, // FGR 2x (tRFC2)
        { 90,   110,  160,  260}, // FGR 4x (tRFC4)
      };

      // tREFI(base) table (unit is nanosecond!)
      constexpr int tREFI_BASE = 7800;
      int density_id = [](int density_Mb) -> int { 
        switch (density_Mb) {
          case 2048:  return 0;
          case 4096:  return 1;
          case 8192:  return 2;
          case 16384: return 3;
          default:    return -1;
        }
      }(m_organization.density);

      m_timing_vals("nRFC")  = JEDEC_rounding(tRFC_TABLE[0][density_id], tCK_ps);
      m_timing_vals("nREFI") = JEDEC_rounding(tREFI_BASE, tCK_ps);

      // Overwrite timing parameters with any user-provided value
      // Rate and tCK should not be overwritten
      for (int i = 1; i < m_timings.size() - 1; i++) {
        auto timing_name = std::string(m_timings(i));

        if (auto provided_timing = param_group("timing").param<int>(timing_name).optional()) {
          // Check if the user specifies in the number of cycles (e.g., nRCD)
          m_timing_vals(i) = *provided_timing;
        } else if (auto provided_timing = param_group("timing").param<float>(timing_name.replace(0, 1, "t")).optional()) {
          // Check if the user specifies in nanoseconds (e.g., tRCD)
          m_timing_vals(i) = JEDEC_rounding(*provided_timing, tCK_ps);
        }
      }

      // Check if there is any uninitialized timings
      for (int i = 0; i < m_timing_vals.size(); i++) {
        if (m_timing_vals(i) == -1) {
          throw ConfigurationError("In \"{}\", timing {} is not specified!", get_name(), m_timings(i));
        }
      }      

      // Set read latency
      m_read_latency = m_timing_vals("nCL") + m_timing_vals("nBL");

      // Populate the timing constraints
      #define V(timing) (m_timing_vals(timing))
      populate_timingcons(this, {
          /*** Channel ***/ 
          // CAS <-> CAS
          /// Data bus occupancy
          // No need for PuM commands, since no data bus occupancy
          {.level = "channel", .preceding = {"RD", "RDA"}, .following = {"RD", "RDA"}, .latency = V("nBL")},
          {.level = "channel", .preceding = {"WR", "WRA"}, .following = {"WR", "WRA"}, .latency = V("nBL")},

          /*** Rank (or different BankGroup) ***/ 
          // CAS <-> CAS
          /// nCCDS is the minimal latency for column commands
          //// No PuM column commands latency since there is no specific column targeted for PuM
          {.level = "rank", .preceding = {"RD", "RDA"}, .following = {"RD", "RDA"}, .latency = V("nCCDS")},
          {.level = "rank", .preceding = {"WR", "WRA"}, .following = {"WR", "WRA"}, .latency = V("nCCDS")},
          /// RD <-> WR, Minimum Read to Write, Assuming tWPRE = 1 tCK
          //// No difference for PuM commands, since we dont need to add switching time                          
          {.level = "rank", .preceding = {"RD", "RDA"}, .following = {"WR", "WRA"}, .latency = V("nCL") + V("nBL") + 2 - V("nCWL")},
          /// WR <-> RD, Minimum Read after Write
          {.level = "rank", .preceding = {"WR", "WRA"}, .following = {"RD", "RDA"}, .latency = V("nCWL") + V("nBL") + V("nWTRS")},
          /// CAS <-> CAS between sibling ranks, nCS (rank switching) is needed for new DQS
          {.level = "rank", .preceding = {"RD", "RDA"}, .following = {"RD", "RDA", "WR", "WRA"}, .latency = V("nBL") + V("nCS"), .is_sibling = true},
          {.level = "rank", .preceding = {"WR", "WRA"}, .following = {"RD", "RDA"}, .latency = V("nCL")  + V("nBL") + V("nCS") - V("nCWL"), .is_sibling = true},
          // Need rank switching time for PuM but dont need the burst length
          {.level = "rank", .preceding = {"FRAC", "MAJ", "RC"}, .following = {"RD", "RDA", "WR", "WRA", "FRAC", "MAJ", "RC"}, .latency = V("nCS"), .is_sibling = true},
          {.level = "rank", .preceding = {"RD", "RDA", "WR", "WRA"}, .following = {"FRAC", "MAJ", "RC"}, .latency = V("nCS"), .is_sibling = true},
          /// CAS <-> PREab
          // No additional time all PuM already ends in PRE
          {.level = "rank", .preceding = {"RD"}, .following = {"PREA"}, .latency = V("nRTP")},
          {.level = "rank", .preceding = {"WR"}, .following = {"PREA"}, .latency = V("nCWL") + V("nBL") + V("nWR")},         
          {.level = "rank", .preceding = {"FRAC", "RC", "MAJ"}, .following = {"PREA"}, .latency = 1},

          /// RAS <-> RAS
          {.level = "rank", .preceding = {"ACT"}, .following = {"ACT", "ACTp", "ACTv"}, .latency = V("nRRDS")},         
          {.level = "rank", .preceding = {"ACT"}, .following = {"ACT"}, .latency = V("nFAW"), .window = 4},          
          {.level = "rank", .preceding = {"ACT"}, .following = {"PREA"}, .latency = V("nRAS")},          
          {.level = "rank", .preceding = {"PREA"}, .following = {"ACT", "ACTp"}, .latency = V("nRP")},
          // Add the PuM acts for the bank prallelism
          {.level = "rank", .preceding = {"ACTp"}, .following = {"ACT", "ACTp", "ACTv"}, .latency = V("nRRDS")},
          {.level = "rank", .preceding = {"ACTv"}, .following = {"ACT", "ACTp", "ACTv"}, .latency = V("nRRDS")},  

          /// RAS <-> REF
          {.level = "rank", .preceding = {"ACT"}, .following = {"REFab"}, .latency = V("nRC")},          
          {.level = "rank", .preceding = {"PRE", "PREA"}, .following = {"REFab"}, .latency = V("nRP")},          
          {.level = "rank", .preceding = {"RDA"}, .following = {"REFab"}, .latency = V("nRP") + V("nRTP")},          
          {.level = "rank", .preceding = {"WRA"}, .following = {"REFab"}, .latency = V("nCWL") + V("nBL") + V("nWR") + V("nRP")},          
          {.level = "rank", .preceding = {"REFab"}, .following = {"ACT", "PREA"}, .latency = V("nRFC")},          
          // Using the nRC timing, since all PuM commands are faster than it anyways
          {.level = "rank", .preceding = {"ACTp"}, .following = {"REFab"}, .latency = V("nRC")},          
          
          /*** Same Bank Group ***/ 
          /// CAS <-> CAS
          {.level = "bankgroup", .preceding = {"RD", "RDA"}, .following = {"RD", "RDA"}, .latency = V("nCCDL")},          
          {.level = "bankgroup", .preceding = {"WR", "WRA"}, .following = {"WR", "WRA"}, .latency = V("nCCDL")},          
          {.level = "bankgroup", .preceding = {"WR", "WRA"}, .following = {"RD", "RDA"}, .latency = V("nCWL") + V("nBL") + V("nWTRL")},
          // Only care about the bankgroup delay to final PuM commands followed by access command, since the preceeding end PuM commands are not really commands
          // They are just a place holder for how long you need to backoff to enforce precharge timing delays correctly
          // So you can issue them as soon as they are possible, but everything after it needs to be issued with constraints
          {.level = "bankgroup", .preceding = {"MAJ", "FRAC", "RC"}, .following = {"RD", "RDA", "WR", "WRA"}, .latency = V("nCCDL")},          
          
          /// RAS <-> RAS
          {.level = "bankgroup", .preceding = {"ACT"}, .following = {"ACT", "ACTp", "ACTv"}, .latency = V("nRRDL")}, 
          {.level = "bankgroup", .preceding = {"ACTp"}, .following = {"ACT", "ACTp", "ACTv"}, .latency = V("nRRDL")},
          {.level = "bankgroup", .preceding = {"ACTv"}, .following = {"ACT", "ACTp", "ACTv"}, .latency = V("nRRDL")},

          /*** Bank ***/ 
          {.level = "bank", .preceding = {"ACT"}, .following = {"ACT"}, .latency = V("nRC")},
          {.level = "bank", .preceding = {"ACT"}, .following = {"RD", "RDA", "WR", "WRA"}, .latency = V("nRCD")},  
          {.level = "bank", .preceding = {"ACT"}, .following = {"PRE"}, .latency = V("nRAS")},  
          {.level = "bank", .preceding = {"PRE"}, .following = {"ACT"}, .latency = V("nRP")},  
          {.level = "bank", .preceding = {"RD"},  .following = {"PRE"}, .latency = V("nRTP")},  
          {.level = "bank", .preceding = {"WR"},  .following = {"PRE"}, .latency = V("nCWL") + V("nBL") + V("nWR")},  
          {.level = "bank", .preceding = {"RDA"}, .following = {"ACT"}, .latency = V("nRTP") + V("nRP")},  
          {.level = "bank", .preceding = {"WRA"}, .following = {"ACT"}, .latency = V("nCWL") + V("nBL") + V("nWR") + V("nRP")},
          
          // Additional delays for PuM
          /// Since all PuM commands have been implemented to be complete in themselves, everything can be issued after they are done
          /// They are done after the main timings have passed (See below PuM main timings)
          {.level = "bank", .preceding = {"RC"}, .following = {"RD", "WR", "ACT", "PRE", "WRA", "RDA", "WR", "FRAC", "MAJ", "RC"}, .latency = V("nRAS") + 6 + V("nRP")},
          {.level = "bank", .preceding = {"MAJ"}, .following = {"RD", "WR", "ACT", "PRE", "WRA", "RDA", "WR", "FRAC", "RC", "MAJ"}, .latency = 3 + 6 + V("nRP")},
          {.level = "bank", .preceding = {"FRAC"}, .following = {"RD", "WR", "ACT", "PRE", "WRA", "RDA", "WR", "RC", "MAJ", "FRAC"}, .latency = 1 + V("nRP")},

          // PuM main timings
          /// ROWCLONE:   ACT tRAS -> PREv 3ns -> ACTv tRP-> RC (Need to manually issue a PRE after)
          /// MAJORITY:   ACT 1.5ns -> PREj 3ns -> ACTv tRP -> MAJ (Back in closed state again)
          /// FRACTIONAL: ACT 1 cylce -> PREf tRP -> FRAC (Back in closed state again)
          
          // Rowclone Command timings
          {.level = "bank", .preceding = {"ACTp"}, .following = {"PREv"}, .latency = V("nRAS")},
          {.level = "bank", .preceding = {"PREv"}, .following = {"ACTv"}, .latency = 6},
          {.level = "bank", .preceding = {"ACTv"}, .following = {"RC"}, .latency = V("nRP")},
          
          // MAJORITY Command timings
          {.level = "bank", .preceding = {"ACTp"}, .following = {"PREj"}, .latency = 3},
          {.level = "bank", .preceding = {"PREj"}, .following = {"ACTv"}, .latency = 6},
          {.level = "bank", .preceding = {"ACTv"}, .following = {"MAJ"}, .latency = V("nRP")},

          // FRAC Command timings
          {.level = "bank", .preceding = {"ACTp"}, .following = {"PREf"}, .latency = 1},
          {.level = "bank", .preceding = {"PREf"}, .following = {"FRAC"}, .latency = V("nRP")},
        }
      );
      #undef V

    };

    void set_actions() {
      m_actions.resize(m_levels.size(), std::vector<ActionFunc_t<Node>>(m_commands.size()));

      // Rank Actions
      m_actions[m_levels["rank"]][m_commands["PREA"]] = Lambdas::Action::Rank::PREab<DDR4>;
      m_actions[m_levels["rank"]][m_commands["REFab"]] = Lambdas::Action::Rank::REFab<DDR4>;
      m_actions[m_levels["rank"]][m_commands["REFab_end"]] = Lambdas::Action::Rank::REFab_end<DDR4>;

      // Bank actions
      m_actions[m_levels["bank"]][m_commands["ACT"]] = Lambdas::Action::Bank::ACT<DDR4>;
      m_actions[m_levels["bank"]][m_commands["PRE"]] = Lambdas::Action::Bank::PRE<DDR4>;
      m_actions[m_levels["bank"]][m_commands["RDA"]] = Lambdas::Action::Bank::PRE<DDR4>;
      m_actions[m_levels["bank"]][m_commands["WRA"]] = Lambdas::Action::Bank::PRE<DDR4>;

      // Add PuM bank actions
      m_actions[m_levels["bank"]][m_commands["FRAC"]] = Lambdas::Action::Bank::FRAC<DDR4>;
      m_actions[m_levels["bank"]][m_commands["MAJ"]]  = Lambdas::Action::Bank::MAJ<DDR4>;
      m_actions[m_levels["bank"]][m_commands["RC"]]   = Lambdas::Action::Bank::RC<DDR4>;
      m_actions[m_levels["bank"]][m_commands["ACTp"]] = Lambdas::Action::Bank::ACTp<DDR4>;
      m_actions[m_levels["bank"]][m_commands["ACTv"]] = Lambdas::Action::Bank::ACTv<DDR4>;
      m_actions[m_levels["bank"]][m_commands["PREv"]] = Lambdas::Action::Bank::PREv<DDR4>;
      m_actions[m_levels["bank"]][m_commands["PREj"]] = Lambdas::Action::Bank::PREj<DDR4>;
      m_actions[m_levels["bank"]][m_commands["PREf"]] = Lambdas::Action::Bank::PREf<DDR4>;
    };

    void set_preqs() {
      m_preqs.resize(m_levels.size(), std::vector<PreqFunc_t<Node>>(m_commands.size()));

      // Rank Actions
      m_preqs[m_levels["rank"]][m_commands["REFab"]] = Lambdas::Preq::Rank::RequireAllBanksClosed<DDR4>;

      // Bank actions
      m_preqs[m_levels["bank"]][m_commands["RD"]] = Lambdas::Preq::Bank::RequireRowOpen<DDR4>;
      m_preqs[m_levels["bank"]][m_commands["WR"]] = Lambdas::Preq::Bank::RequireRowOpen<DDR4>;
      m_preqs[m_levels["bank"]][m_commands["ACT"]] = Lambdas::Preq::Bank::RequireRowOpen<DDR4>;
      m_preqs[m_levels["bank"]][m_commands["PRE"]] = Lambdas::Preq::Bank::RequireBankClosed<DDR4>;

      // Add PuM Bank requirements (State machine logic)
      m_preqs[m_levels["bank"]][m_commands["RC"]] = Lambdas::Preq::Bank::RequireRC<DDR4>;
      m_preqs[m_levels["bank"]][m_commands["MAJ"]] = Lambdas::Preq::Bank::RequireMAJ<DDR4>;
      m_preqs[m_levels["bank"]][m_commands["FRAC"]] = Lambdas::Preq::Bank::RequireFRAC<DDR4>;
    };

    void set_rowhits() {
      m_rowhits.resize(m_levels.size(), std::vector<RowhitFunc_t<Node>>(m_commands.size()));

      m_rowhits[m_levels["bank"]][m_commands["RD"]] = Lambdas::RowHit::Bank::RDWR<DDR4>;
      m_rowhits[m_levels["bank"]][m_commands["WR"]] = Lambdas::RowHit::Bank::RDWR<DDR4>;
    }


    void set_rowopens() {
      m_rowopens.resize(m_levels.size(), std::vector<RowhitFunc_t<Node>>(m_commands.size()));

      m_rowopens[m_levels["bank"]][m_commands["RD"]] = Lambdas::RowOpen::Bank::RDWR<DDR4>;
      m_rowopens[m_levels["bank"]][m_commands["WR"]] = Lambdas::RowOpen::Bank::RDWR<DDR4>;
    }

    void set_powers() {
      
      m_drampower_enable = param<bool>("drampower_enable").default_val(false);

      if (!m_drampower_enable)
        return;

      m_voltage_vals.resize(m_voltages.size(), -1);

      if (auto preset_name = param_group("voltage").param<std::string>("preset").optional()) {
        if (voltage_presets.count(*preset_name) > 0) {
          m_voltage_vals = voltage_presets.at(*preset_name);
        } else {
          throw ConfigurationError("Unrecognized voltage preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      m_current_vals.resize(m_currents.size(), -1);

      if (auto preset_name = param_group("current").param<std::string>("preset").optional()) {
        if (current_presets.count(*preset_name) > 0) {
          m_current_vals = current_presets.at(*preset_name);
        } else {
          throw ConfigurationError("Unrecognized current preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      m_power_debug = param<bool>("power_debug").default_val(false);

      // TODO: Check for multichannel configs.
      int num_channels = m_organization.count[m_levels["channel"]];
      int num_ranks = m_organization.count[m_levels["rank"]];
      m_power_stats.resize(num_channels * num_ranks);
      for (int i = 0; i < num_channels; i++) {
        for (int j = 0; j < num_ranks; j++) {
          m_power_stats[i * num_ranks + j].rank_id = i * num_ranks + j;
          m_power_stats[i * num_ranks + j].cmd_counters.resize(m_cmds_counted.size(), 0);
        }
      }

      m_powers.resize(m_levels.size(), std::vector<PowerFunc_t<Node>>(m_commands.size()));

      m_powers[m_levels["bank"]][m_commands["ACT"]] = Lambdas::Power::Bank::ACT<DDR4>;
      m_powers[m_levels["bank"]][m_commands["PRE"]] = Lambdas::Power::Bank::PRE<DDR4>;
      m_powers[m_levels["bank"]][m_commands["RD"]]  = Lambdas::Power::Bank::RD<DDR4>;
      m_powers[m_levels["bank"]][m_commands["WR"]]  = Lambdas::Power::Bank::WR<DDR4>;

      m_powers[m_levels["rank"]][m_commands["ACT"]] = Lambdas::Power::Rank::ACT<DDR4>;
      m_powers[m_levels["rank"]][m_commands["PRE"]] = Lambdas::Power::Rank::PRE<DDR4>;
      m_powers[m_levels["rank"]][m_commands["PREA"]] = Lambdas::Power::Rank::PREA<DDR4>;
      m_powers[m_levels["rank"]][m_commands["REFab"]] = Lambdas::Power::Rank::REFab<DDR4>;
      m_powers[m_levels["rank"]][m_commands["REFab_end"]] = Lambdas::Power::Rank::REFab_end<DDR4>;

      // register stats
      register_stat(s_total_background_energy).name("total_background_energy");
      register_stat(s_total_cmd_energy).name("total_cmd_energy");
      register_stat(s_total_energy).name("total_energy");
            
      for (auto& power_stat : m_power_stats){
        register_stat(power_stat.total_background_energy).name("total_background_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.total_cmd_energy).name("total_cmd_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.total_energy).name("total_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.act_background_energy).name("act_background_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.pre_background_energy).name("pre_background_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.active_cycles).name("active_cycles_rank{}", power_stat.rank_id);
        register_stat(power_stat.idle_cycles).name("idle_cycles_rank{}", power_stat.rank_id);
      }
    }

    void create_nodes() {
      int num_channels = m_organization.count[m_levels["channel"]];
      for (int i = 0; i < num_channels; i++) {
        Node* channel = new Node(this, nullptr, 0, i);
        m_channels.push_back(channel);
      }
    }

    void finalize() override {
      if (!m_drampower_enable)
        return;

      int num_channels = m_organization.count[m_levels["channel"]];
      int num_ranks = m_organization.count[m_levels["rank"]];
      for (int i = 0; i < num_channels; i++) {
        for (int j = 0; j < num_ranks; j++) {
          process_rank_energy(m_power_stats[i * num_ranks + j], m_channels[i]->m_child_nodes[j]);
        }
      }
    }

    void process_rank_energy(PowerStats& rank_stats, Node* rank_node) {
      
      Lambdas::Power::Rank::finalize_rank<DDR4>(rank_node, 0, AddrVec_t(), m_clk);

      auto TS = [&](std::string_view timing) { return m_timing_vals(timing); };
      auto VE = [&](std::string_view voltage) { return m_voltage_vals(voltage); };
      auto CE = [&](std::string_view current) { return m_current_vals(current); };

      double tCK_ns = (double) TS("tCK_ps") / 1000.0;

      rank_stats.act_background_energy = (VE("VDD") * CE("IDD3N") + VE("VPP") * CE("IPP3N")) 
                                            * rank_stats.active_cycles * tCK_ns / 1E3;

      rank_stats.pre_background_energy = (VE("VDD") * CE("IDD2N") + VE("VPP") * CE("IPP2N")) 
                                            * rank_stats.idle_cycles * tCK_ns / 1E3;


      double act_cmd_energy  = (VE("VDD") * (CE("IDD0") - CE("IDD3N")) + VE("VPP") * (CE("IPP0") - CE("IPP3N"))) 
                                      * rank_stats.cmd_counters[m_cmds_counted("ACT")] * TS("nRAS") * tCK_ns / 1E3;

      double pre_cmd_energy  = (VE("VDD") * (CE("IDD0") - CE("IDD2N")) + VE("VPP") * (CE("IPP0") - CE("IPP2N"))) 
                                      * rank_stats.cmd_counters[m_cmds_counted("PRE")] * TS("nRP")  * tCK_ns / 1E3;

      double rd_cmd_energy   = (VE("VDD") * (CE("IDD4R") - CE("IDD3N")) + VE("VPP") * (CE("IPP4R") - CE("IPP3N"))) 
                                      * rank_stats.cmd_counters[m_cmds_counted("RD")] * TS("nBL") * tCK_ns / 1E3;

      double wr_cmd_energy   = (VE("VDD") * (CE("IDD4W") - CE("IDD3N")) + VE("VPP") * (CE("IPP4W") - CE("IPP3N"))) 
                                      * rank_stats.cmd_counters[m_cmds_counted("WR")] * TS("nBL") * tCK_ns / 1E3;

      double ref_cmd_energy  = (VE("VDD") * (CE("IDD5B")) + VE("VPP") * (CE("IPP5B"))) 
                                      * rank_stats.cmd_counters[m_cmds_counted("REF")] * TS("nRFC") * tCK_ns / 1E3;

      rank_stats.total_background_energy = rank_stats.act_background_energy + rank_stats.pre_background_energy;
      rank_stats.total_cmd_energy = act_cmd_energy 
                                    + pre_cmd_energy 
                                    + rd_cmd_energy
                                    + wr_cmd_energy 
                                    + ref_cmd_energy;

      rank_stats.total_energy = rank_stats.total_background_energy + rank_stats.total_cmd_energy;

      s_total_background_energy += rank_stats.total_background_energy;
      s_total_cmd_energy += rank_stats.total_cmd_energy;
      s_total_energy += rank_stats.total_energy;
    }
};


}        // namespace Ramulator
