#ifndef RAMULATOR_DRAM_NODE_H
#define RAMULATOR_DRAM_NODE_H

#include <vector>
#include <map>
#include <deque>
#include <functional>
#include <concepts>

#include "base/type.h"
#include "dram/spec.h"

namespace Ramulator {

class IDRAM;

template<typename T>
concept IsDRAMSpec = requires(T t) { 
  typename T::Node; 
};

// CRTP class defnition is not complete, so we cannot have something nice like:
// template<typename T>
// concept IsDRAMSpec = std::is_base_of_v<IDRAM, T> && requires(T t) { 
//   typename T::Node; 
// };


/**
 * @brief     CRTP-ish (?) base class of a DRAM Device Node
 * 
 */
template<IsDRAMSpec T>
struct DRAMNodeBase {
    using NodeType = typename T::Node;
    NodeType* m_parent_node = nullptr;
    std::vector<NodeType*> m_child_nodes;

    T* m_spec = nullptr;

    int m_level = -1;      // The level of this node in the organization hierarchy
    int m_node_id = -1;    // The id of this node at this level
    int m_size = -1;       // The size of the node (e.g., how many rows in a bank)

    int m_state = -1;      // The state of the node

    std::vector<Clk_t> m_cmd_ready_clk;             // The next cycle that each command can be issued again at this level
    std::vector<std::deque<Clk_t>> m_cmd_history;   // Issue-history of each command at this level

    using RowId_t = int;
    using RowState_t = int;
    std::map<RowId_t, RowState_t> m_row_state;  // The state of the rows, if I am a bank-ish node

    DRAMNodeBase(T* spec, NodeType* parent, int level, int id):
    m_spec(spec), m_parent_node(parent), m_level(level), m_node_id(id) {
      int num_cmds = T::m_commands.size();
      m_cmd_ready_clk.resize(num_cmds, -1);
      m_cmd_history.resize(num_cmds);
      for (int cmd = 0; cmd < num_cmds; cmd++) {
        int window = 0;
        for (const auto& t : spec->m_timing_cons[level][cmd]) {
          window = std::max(window, t.window);
        }
        if (window != 0) {
          m_cmd_history[cmd].resize(window, -1);
        } else {
          m_cmd_history[cmd].clear();
        }
      }

      m_state = spec->m_init_states[m_level];

      // Recursively construct next levels
      int next_level = level + 1;
      int last_level = T::m_levels["row"];
      if (next_level == last_level) {
        return;
      } else {
        int next_level_size = m_spec->m_organization.count[next_level];
        if (next_level_size == 0) {
          return;
        } else {
          for (int i = 0; i < next_level_size; i++) {
            NodeType* child = new NodeType(spec, static_cast<NodeType*>(this), next_level, i);
            static_cast<NodeType*>(this)->m_child_nodes.push_back(child);
          }
        }
      }
    };

    void update_states(int command, const AddrVec_t& addr_vec, Clk_t clk) {
      int child_id = addr_vec[m_level+1];
      if (m_spec->m_actions[m_level][command]) {
        // update the state machine at this level
        m_spec->m_actions[m_level][command](static_cast<NodeType*>(this), command, child_id, clk); 
      }
      if (m_level == m_spec->m_command_scopes[command] || !m_child_nodes.size()) {
        // stop recursion: updated all levels
        return; 
      }
      // recursively update child nodes
      if (child_id == -1) {
        for (auto child : m_child_nodes) {
          child->update_states(command, addr_vec, clk);
        }
      } else {
        m_child_nodes[child_id]->update_states(command, addr_vec, clk);
      }
    };

    void update_powers(int command, const AddrVec_t& addr_vec, Clk_t clk) {
      if (!m_spec->m_drampower_enable)
        return;

      int child_id = addr_vec[m_level+1];
      if (m_spec->m_powers[m_level][command]) {
        // update the power model at this level
        m_spec->m_powers[m_level][command](static_cast<NodeType*>(this), command, addr_vec, clk);
      }
      if (m_level == m_spec->m_command_scopes[command] || !m_child_nodes.size()) {
        // stop recursion: updated all levels
        return; 
      }
      // recursively update child nodes
      if (child_id == -1){
        for (auto child : m_child_nodes) {
          child->update_powers(command, addr_vec, clk);
        }
      } else {
        m_child_nodes[child_id]->update_powers(command, addr_vec, clk);
      }
    };

    void update_timing(int command, const AddrVec_t& addr_vec, Clk_t clk) {
      /************************************************
       *         Update Sibling Node Timing
       ***********************************************/
      if (m_node_id != addr_vec[m_level] && addr_vec[m_level] != -1) {
        for (const auto& t : m_spec->m_timing_cons[m_level][command]) {
          if (!t.sibling) {
            // not sibling timing parameter
            continue; 
          }

          // update earliest schedulable time of every command
          Clk_t future = clk + t.val;
          m_cmd_ready_clk[t.cmd] = std::max(m_cmd_ready_clk[t.cmd], future); 
        }
        // stop recursion
        return;
      }

      /************************************************
       *          Update Target Node Timing
       ***********************************************/
      // Update history
      if (m_cmd_history[command].size()) {
        m_cmd_history[command].pop_back();
        m_cmd_history[command].push_front(clk); 
      }

      for (const auto& t : m_spec->m_timing_cons[m_level][command]) {
        if (t.sibling) {
          continue; 
        }

        // Get the oldest history
        Clk_t past = m_cmd_history[command][t.window-1];
        if (past < 0) {
          // not enough history
          continue; 
        }

        // update earliest schedulable time of every command
        Clk_t future = past + t.val;
        m_cmd_ready_clk[t.cmd] = std::max(m_cmd_ready_clk[t.cmd], future);
      }

      if (!m_child_nodes.size()) {
        // stop recursion: updated all levels
        return; 
      }

      // recursively update all of my children
      for (auto child : m_child_nodes) {
        child->update_timing(command, addr_vec, clk);
      }
    };

    int get_preq_command(int command, const AddrVec_t& addr_vec, Clk_t m_clk) {
      int child_id = addr_vec[m_level + 1];
      if (m_spec->m_preqs[m_level][command]) {
        int preq_cmd = m_spec->m_preqs[m_level][command](static_cast<NodeType*>(this), command, addr_vec, m_clk);
        if (preq_cmd != -1) {
          // stop recursion: there is a prerequisite at this level
          return preq_cmd; 
        }
      }

      if (!m_child_nodes.size()) {
        // stop recursion: there were no prequisites at any level, thus return -1, since no prerequisite
        // Not sure if there is any command in DRAM that does not have any prerequisits so the default return command wouldnt make any sense either
        return -1;
      }

      // recursively get_preq_command at my child
      return m_child_nodes[child_id]->get_preq_command(command, addr_vec, m_clk);
    };

    // Function that returns the ready clock for the command passed
    // Usage is exclusive for already in the active buffer commands, all else might behave ujnexpectelty due to the command being prerequisite command being -1  
    int get_ready_clk_active(int command, const AddrVec_t& addr_vec, Clk_t clk) {
      // it should not be possible for command to be -1 at this, if it was we would be hard stuck on this bank
      if (command != -1 && clk < m_cmd_ready_clk[command]) {
        // stop recursion: the check failed at this level
        return m_cmd_ready_clk[command];
      }

      int child_id = addr_vec[m_level+1];
      if (m_level == m_spec->m_command_scopes[command] || !m_child_nodes.size()) {
        // stop recursion: the check passed at all levels
        return m_cmd_ready_clk[command]; 
      }

      if (child_id == -1) {
        // if it is a same bank command, recurse all children in rank level
        bool ready = true;
        for (auto child : m_child_nodes) {
          ready = ready && child->check_ready(command, addr_vec, clk);
        }
        return m_cmd_ready_clk[command];
      } else {
        // recursively check my child
        return m_child_nodes[child_id]->check_ready(command, addr_vec, clk);
      }
    }

    bool check_ready(int command, const AddrVec_t& addr_vec, Clk_t clk) {
      // They didnt add that if the command was actually -1 what should happen.
      // With new logic it is an expected case and should issue that command is not ready since -1 doesnt exist and cant exist
      if (command == -1 || (m_cmd_ready_clk[command] != -1 && clk < m_cmd_ready_clk[command])) {
        // stop recursion: the check failed at this level
        return false; 
      }

      int child_id = addr_vec[m_level+1];
      if (m_level == m_spec->m_command_scopes[command] || !m_child_nodes.size()) {
        // stop recursion: the check passed at all levels
        return true; 
      }

      if (child_id == -1) {
        // if it is a same bank command, recurse all children in rank level
        bool ready = true;
        for (auto child : m_child_nodes) {
          ready = ready && child->check_ready(command, addr_vec, clk);
        }
        return ready;
      } else {
        // recursively check my child
        return m_child_nodes[child_id]->check_ready(command, addr_vec, clk);
      }
    };

    bool check_rowbuffer_hit(int command, const AddrVec_t& addr_vec, Clk_t m_clk) {
      // TODO: Optimize this by just checking the bank-levels? Have a dedicated bank structure?
      int child_id = addr_vec[m_level+1];
      if (m_spec->m_rowhits[m_level][command]) {
        // stop recursion: there is a row hit at this level
        return m_spec->m_rowhits[m_level][command](static_cast<NodeType*>(this), command, child_id, m_clk);  
      }

      if (!m_child_nodes.size()) {
        // stop recursion: there were no row hits at any level
        return false; 
      }

      // recursively check for row hits at my child
      return m_child_nodes[child_id]->check_rowbuffer_hit(command, addr_vec, m_clk);
    };    
    
    bool check_node_open(int command, const AddrVec_t& addr_vec, Clk_t m_clk) {

      int child_id = addr_vec[m_level+1];
      if (m_spec->m_rowopens[m_level][command])
        // stop recursion: there is a row open at this level
        return m_spec->m_rowopens[m_level][command](static_cast<NodeType*>(this), command, child_id, m_clk);  

      if (!m_child_nodes.size())
        // stop recursion: there were no row hits at any level
        return false; 

      // recursively check for row hits at my child
      return m_child_nodes[child_id]->check_node_open(command, addr_vec, m_clk);
    }
};

template<class T>
using ActionFunc_t = std::function<void(typename T::Node* node, int cmd, int target_id, Clk_t clk)>;
template<class T>
using PreqFunc_t   = std::function<int (typename T::Node* node, int cmd, const AddrVec_t& addr_vec, Clk_t clk)>;
template<class T>
using RowhitFunc_t = std::function<bool(typename T::Node* node, int cmd, int target_id, Clk_t clk)>;
template<class T>
using RowopenFunc_t = std::function<bool(typename T::Node* node, int cmd, int target_id, Clk_t clk)>;
template<class T>
using PowerFunc_t = std::function<void(typename T::Node* node, int cmd, const AddrVec_t& addr_vec, Clk_t clk)>;

template<typename T>
using FuncMatrix = std::vector<std::vector<T>>;

// TODO: Enable easy syntax for FuncMatrix lookup
// template<typename T, int N, int M>
// class FuncMatrix : public std::array<std::array<T, M>, N> {

// };

}        // namespace Ramulator

#endif   // RAMULATOR_DRAM_NODE_H
