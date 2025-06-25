#ifndef RAMULATOR_DRAM_LAMBDAS_ACTION_H
#define RAMULATOR_DRAM_LAMBDAS_ACTION_H

#include <spdlog/spdlog.h>

#include "dram/node.h"

namespace Ramulator {
namespace Lambdas {

template<class>
inline constexpr bool false_v = false;

namespace Action {
namespace Bank {
  template <class T>
  void ACT(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    node->m_state = T::m_states["Opened"];
    node->m_row_state[target_id] = T::m_states["Opened"];
  };

  template <class T>
  void PRE(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    node->m_state = T::m_states["Closed"];
    node->m_row_state.clear();
  };

  // Closed ----> OpenedPum
  template <class T>
  void ACTp(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    node->m_state = T::m_states["OpenedPum"];
    node->m_row_state[target_id] = T::m_states["OpenedPum"];
  };

  // For RC: Closed, OpenedPum, RCState, Processed, Closed
  template <class T>
  void PREv(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    node->m_state = T::m_states["RCState"];
    node->m_row_state[target_id] = T::m_states["RCState"];
  };
  
  template <class T>
  void ACTv(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    node->m_state = T::m_states["Processed"];
    node->m_row_state[target_id] = T::m_states["Processed"];
  };

  template <class T>
  void RC(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    node->m_state = T::m_states["Closed"];
    node->m_row_state.clear();
  };

  // For MAJ: Closed, OpenedPum, MAJState, Processed, Closed
  template <class T>
  void PREj(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    node->m_state = T::m_states["MAJState"];
    node->m_row_state[target_id] = T::m_states["MAJState"];
  };

  template <class T>
  void MAJ(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    node->m_state = T::m_states["Closed"];
    node->m_row_state.clear();
  };

  // For FRAC: Closed, OpenedPum, Processedm Closed
  template <class T>
  void PREf(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    node->m_state = T::m_states["Processed"];
    node->m_row_state[target_id] = T::m_states["Processed"];
  };

  template <class T>
  void FRAC(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    node->m_state = T::m_states["Closed"];
    node->m_row_state.clear();
  };

  template <class T>
  void VRR(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    node->m_state = T::m_states["Refreshing"];
  };

  template <class T>
  void VRR_end(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    node->m_state = T::m_states["Closed"];
  };

}       // namespace Bank

namespace BankGroup {
template <class T>
  void PREsb(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    for (auto bank : node->m_child_nodes) {
      if (bank->m_node_id == target_id) {
        bank->m_state = T::m_states["Closed"];
        bank->m_row_state.clear();
      }
    }
  };


  template <class T>
  void REFsb(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    for (auto bank : node->m_child_nodes) {
      if (bank->m_node_id == target_id) {
        bank->m_state = T::m_states["Refreshing"];
      }
    }
  }

  template <class T>
  void REFsb_end(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    for (auto bank : node->m_child_nodes) {
      if (bank->m_node_id == target_id) {
        bank->m_state = T::m_states["Closed"];
        bank->m_row_state.clear();
      }
    }
  }
}       // namespace BankGroup

namespace Rank {
  template <class T>
  void PREab(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    if constexpr (T::m_levels["bank"] - T::m_levels["rank"] == 1) {
      for (auto bank : node->m_child_nodes) {
        bank->m_state = T::m_states["Closed"];
        bank->m_row_state.clear();
      }
    } else if constexpr (T::m_levels["bank"] - T::m_levels["rank"] == 2) {
      for (auto bg : node->m_child_nodes) {
        for (auto bank : bg->m_child_nodes) {
          bank->m_state = T::m_states["Closed"];
          bank->m_row_state.clear();
        }
      }
    } else {
      static_assert(
        false_v<T>, 
        "[Action::Rank] Unsupported organization. Please write your own PREab function."
      );
    }
  };

template <class T>
  void REFab(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    for (auto bg : node->m_child_nodes) {
      for (auto bank : bg->m_child_nodes) {
        bank->m_state = T::m_states["Refreshing"];
      }
    }
  };

  template <class T>
  void REFab_end(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    for (auto bg : node->m_child_nodes) {
      for (auto bank : bg->m_child_nodes) {
        bank->m_state = T::m_states["Closed"];
      }
    }
  };

  }       // namespace Rank

namespace Channel {
  // TODO: Make these nicer...
  template <class T>
  void PREab(typename T::Node* node, int cmd, int target_id, Clk_t clk) {
    if constexpr (T::m_levels["bank"] - T::m_levels["channel"] == 2) {
      for (auto bg : node->m_child_nodes) {
        for (auto bank : bg->m_child_nodes) {
          bank->m_state = T::m_states["Closed"];
          bank->m_row_state.clear();
        }
      }
    } else if constexpr (T::m_levels["bank"] - T::m_levels["channel"] == 3) {
      for (auto pc : node->m_child_nodes) {
        for (auto bg : pc->m_child_nodes) {
          for (auto bank : bg->m_child_nodes) {
            bank->m_state = T::m_states["Closed"];
            bank->m_row_state.clear();
          }
        }
      }
    } else {
      static_assert(
        false_v<T>, 
        "[Action::Rank] Unsupported organization. Please write your own PREab function."
      );
    }
  };
}      // namespace Channel
}       // namespace Action
}       // namespace Lambdas
};      // namespace Ramulator

#endif  // RAMULATOR_DRAM_LAMBDAS_ACTION_H