#ifndef RAMULATOR_DRAM_LAMBDAS_PREQ_H
#define RAMULATOR_DRAM_LAMBDAS_PREQ_H

#include <spdlog/spdlog.h>
#include <vector>
#include <cstdint>

using AddrVec_t = std::vector<int>;
using Clk_t = uint64_t;

namespace Ramulator {
namespace Lambdas {
namespace Preq {
namespace Bank {
template <class T>
int RequireRowOpen(typename T::Node* node, int cmd, const AddrVec_t& addr_vec, Clk_t clk) {
  switch (node->m_state) {
    case T::m_states["Closed"]: return T::m_commands["ACT"];
    case T::m_states["Opened"]: {
      if (node->m_row_state.find(addr_vec[T::m_levels["row"]]) != node->m_row_state.end()) {
        return cmd;
      } else {
        return T::m_commands["PRE"];
      }
    }
    case T::m_states["Refreshing"]: return T::m_commands["ACT"];
    default: {
      return -1;     
    } 
  }
};

template <class T>
int RequireBankClosed(typename T::Node* node, int cmd, const AddrVec_t& addr_vec, Clk_t clk) {
  switch (node->m_state) {
    case T::m_states["Closed"]: return cmd;
    case T::m_states["Opened"]: return T::m_commands["PRE"];
    case T::m_states["Refreshing"]: return cmd;
    default: { // Something is already active and this state is not meant to be interupted so the return wont matter
      return -1;
    } 
  }
};

// Handle all going into PuM state defaults
template <class T>
int RequireRowOpenPum(typename T::Node* node, int cmd, const AddrVec_t& addr_vec, Clk_t clk) {
  switch (node->m_state) {
    // Issue PuM open command if the state is closed
    case T::m_states["Closed"]: return T::m_commands["ACTp"];
    // If a state is already normally opened, PRE to close it
    case T::m_states["Opened"]: return T::m_commands["PRE"];
    // If we are refreshing than just take ACTp since it is a form of refreshing
    case T::m_states["Refreshing"]: return T::m_commands["ACTp"];
    default: {
      return Ramulator::Lambdas::Preq::Bank::RequireBankClosed<T>(node, cmd, addr_vec, clk);
    } 
  }
};

template <class T>
int RequireRC(typename T::Node* node, int cmd, const AddrVec_t& addr_vec, Clk_t clk) {
  switch (node->m_state) {
    // Issue the PREv command for Rowclone APA
    case T::m_states["OpenedPum"]: return T::m_commands["PREv"];
    // issue the RC
    case T::m_states["RCState"]: return cmd;
    // Need to first get into the OPEN PuM state
    default: return Ramulator::Lambdas::Preq::Bank::RequireRowOpenPum<T>(node, cmd, addr_vec, clk);
  }
};

template <class T>
int RequireMAJ(typename T::Node* node, int cmd, const AddrVec_t& addr_vec, Clk_t clk) {
  switch (node->m_state) {
    // Isssue the PREj command for MAJ APA
    case T::m_states["OpenedPum"]: return T::m_commands["PREj"];
    // issue the MAJ
    case T::m_states["MAJState"]: return cmd;
    // Need to first get into the OPEN PuM state
    default: return Ramulator::Lambdas::Preq::Bank::RequireRowOpenPum<T>(node, cmd, addr_vec, clk);
  }
};

template <class T>
int RequireFRAC(typename T::Node* node, int cmd, const AddrVec_t& addr_vec, Clk_t clk) {
  switch (node->m_state) {
    case T::m_states["OpenedPum"]: return cmd;
    // Neeed to first get into the OPEN PuM state
    default: return Ramulator::Lambdas::Preq::Bank::RequireRowOpenPum<T>(node, cmd, addr_vec, clk);
  }
};
}       // namespace Bank

namespace Rank {
template <class T>
int RequireAllBanksClosed(typename T::Node* node, int cmd, const AddrVec_t& addr_vec, Clk_t clk) {
  if constexpr (T::m_levels["bank"] - T::m_levels["rank"] == 1) {
    for (auto bank: node->m_child_nodes) {
      if (bank->m_state == T::m_states["Closed"]) {
        continue;
      } else if(bank->m_state == T::m_states["Refreshing"]) {
        return cmd; 
      } else {
        return T::m_commands["PREA"];
      }
    }
  } else if constexpr (T::m_levels["bank"] - T::m_levels["rank"] == 2) {
    for (auto bg : node->m_child_nodes) {
      for (auto bank: bg->m_child_nodes) {
        if (bank->m_state == T::m_states["Closed"]) {
          continue;
        } else if(bank->m_state == T::m_states["Refreshing"]) {
          return cmd; 
        } else {
          return T::m_commands["PREA"];
        }
      }
    }
  }
  return cmd;
};

template <class T>
int RequireSameBanksClosed(typename T::Node* node, int cmd, const AddrVec_t& addr_vec, Clk_t clk) {
  bool all_banks_ready = true;
  for (auto bg : node->m_child_nodes) {
    for (auto bank : bg->m_child_nodes) {
      if (bank->m_node_id == addr_vec[T::m_levels["bank"]]) {
        all_banks_ready &= (bank->m_state == T::m_states["Closed"]) || (bank->m_state == T::m_states["Refreshing"]);
      }
    }
  }
  if (all_banks_ready) {
    return cmd;
  } else {
    return T::m_commands["PREsb"];
  }
};
}       // namespace Rank
namespace Channel {
  template <class T>
  int RequireAllBanksClosed(typename T::Node* node, int cmd, const AddrVec_t& addr_vec, Clk_t clk) {
    if constexpr (T::m_levels["bank"] - T::m_levels["channel"] == 2) {
      for (auto bg : node->m_child_nodes) {
        for (auto bank: bg->m_child_nodes) {
          if (bank->m_state == T::m_states["Closed"]) {
            continue;
          } else {
            return T::m_commands["PREA"];
          }
        }
      }
    } else if constexpr (T::m_levels["bank"] - T::m_levels["channel"] == 3) {
      for (auto pc : node->m_child_nodes) {
        for (auto bg : pc->m_child_nodes) {
          for (auto bank: bg->m_child_nodes) {
            if (bank->m_state == T::m_states["Closed"]) {
              continue;
            } else {
              return T::m_commands["PREA"];
            }
          }
        }
      }
    }
    return cmd;
  };
}       // namespace Channel
}       // namespace Preq
}       // namespace Lambdas
}       // namespace Ramulator

#endif  // RAMULATOR_DRAM_LAMBDAS_PREQ_H