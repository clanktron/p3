// Copyright 2025 Blaise Tine
//
// Licensed under the Apache License;
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <iomanip>
#include <string.h>
#include <assert.h>
#include <util.h>
#include "types.h"
#include "core.h"
#include "debug.h"
#include "processor_impl.h"

using namespace tinyrv;

void Core::issue() {
  if (issue_queue_->empty())
    return;

  auto& is_data = issue_queue_->data();
  auto instr = is_data.instr;
  auto exe_flags = instr->getExeFlags();

  // check for structial hazards
  if (RS_.full() || ROB_.full()) {
    return;
  }

  uint32_t rs1_data = 0;  // rs1 data obtained from register file or ROB
  uint32_t rs2_data = 0;  // rs2 data obtained from register file or ROB
  int rs1_rsid = -1;      // reservation station id for rs1 (-1 indicates data in already available)
  int rs2_rsid = -1;      // reservation station id for rs2 (-1 indicates data is already available)

  auto rs1 = instr->getRs1();
  auto rs2 = instr->getRs2();

  // get rs1 data
  if (exe_flags.use_rs1) {
    if (RAT_.exists(rs1)) {
      int rob_index = RAT_.get(rs1);
      if (ROB_.get_entry(rob_index).ready) {
        rs1_data = ROB_.get_entry(rob_index).result;
      } else {
        rs1_rsid = RST_[rob_index];
      }
    } else {
      rs1_data = reg_file_.at(rs1);
    }
  }

  // get rs2 data
  if (exe_flags.use_rs2) {
    if (RAT_.exists(rs2)) {
      int rob_index = RAT_.get(rs2);
      if (ROB_.get_entry(rob_index).ready) {
        rs2_data = ROB_.get_entry(rob_index).result;
      } else {
        rs2_rsid = RST_[rob_index];
      }
    } else {
      rs2_data = reg_file_.at(rs2);
    }
  }

  // allocat new ROB entry and obtain its index
  int rob_index = ROB_.allocate(instr);

  // update the RAT mapping if this instruction write to the register file
  if (exe_flags.use_rd) {
    RAT_.set(instr->getRd(), rob_index);
  }

  // issue the instruction to free reservation station
  int rs_index = RS_.issue(rob_index, rs1_rsid, rs2_rsid, rs1_data, rs2_data, instr);

  // update RST mapping
  if (exe_flags.use_rd) {
    RST_[rob_index] = rs_index;
  }

  DT(2, "Issue: " << *instr);

  // pop issue queue
  issue_queue_->pop();
}

void Core::execute() {
  // execute functional units
  for (auto fu : FUs_) {
    fu->execute();
  }

  // find the next functional units that is done executing
  // and push its output result to the common data bus
  // then clear the functional unit.
  // The CDB can only serve one functional unit per cycle
  // HINT: should use CDB_ and FUs_
  for (auto fu : FUs_) {
    if (fu->done() && CDB_.empty()) {
      auto output = fu->get_output();
      CDB_.push(output.result, output.rob_index, output.rs_index);
      fu->clear();
      break;  // Only one FU can use CDB per cycle
    }
  }

  // schedule ready instructions to corresponding functional units
  // iterate through all reservation stations, check if the entry is valid, but not running yet,
  // and its operands are ready, and also make sure that is not locked (LSU case).
  // once a candidate is found, issue the instruction to its corresponding functional unit.
  // HINT: should use RS_ and FUs_
  for (int rs_index = 0; rs_index < (int)RS_.size(); ++rs_index) {
    auto& entry = RS_.get_entry(rs_index);
    if (entry.valid && !entry.running && RS_.operands_ready(rs_index) && !RS_.locked(rs_index)) {
      auto fu_type = entry.instr->getFUType();
      auto& fu = FUs_.at((int)fu_type);
      if (!fu->busy()) {
        fu->issue(entry.instr, entry.rob_index, rs_index, entry.rs1_data, entry.rs2_data);
        entry.running = true;
        break;  // Only schedule one instruction per cycle
      }
    }
  }
}

void Core::writeback() {
  // CDB broadcast
  if (CDB_.empty())
    return;

  auto& cdb_data = CDB_.data();

  // update all reservation stations waiting for operands
  // HINT: use RS::entry_t::update_operands()
  for (int rs_index = 0; rs_index < (int)RS_.size(); ++rs_index) {
    auto& entry = RS_.get_entry(rs_index);
    if (entry.valid) {
      entry.update_operands(cdb_data);
    }
  }

  // free the RS entry associated with this CDB response
  // so that it can be used by other instructions
  RS_.release(cdb_data.rs_index);

  // update ROB
  ROB_.update(cdb_data);

  // clear CDB
  CDB_.pop();

  RS_.dump();
}

void Core::commit() {
  // commit ROB head entry
  if (ROB_.empty())
    return;

  int head_index = ROB_.head_index();
  auto& rob_head = ROB_.get_entry(head_index);

  // check if the head entry is ready to commit
  if (rob_head.ready) {
    auto instr = rob_head.instr;
    auto exe_flags = instr->getExeFlags();

    // If this instruction writes to the register file,
    // (1) update the register file
    // (2) clear the RAT if still pointing to this ROB head
    if (exe_flags.use_rd) {
      reg_file_.at(instr->getRd()) = rob_head.result;
      if (RAT_.exists(instr->getRd()) && RAT_.get(instr->getRd()) == head_index) {
        RAT_.clear(instr->getRd());
      }
    }

    // pop ROB entry
    ROB_.pop();

    DT(2, "Commit: " << *instr);

    assert(perf_stats_.instrs <= fetched_instrs_);
    ++perf_stats_.instrs;

    // handle program termination
    if (exe_flags.is_exit) {
      exited_ = true;
    }
  }

  ROB_.dump();
}