#pragma once

/**
 * Certificate-level sets oracle for tests.
 *
 * C++ port of set_algo_analysis/check_solution.py::check_sets (the
 * harness that caught the pre-fix violations), operating directly on
 * (Instance, Solution) instead of CSV files.
 *
 * Semantics: linearize all item placements — bins in solution order, a
 * bin with copies k expanded as k sequential repetitions of its item
 * node sequence (physical cut order: plate after plate). Placing an
 * item of row r belonging to set S is legal iff no OTHER row r' of S
 * is active, where active := 0 < cnt[r'] < copies(r') and
 * cnt[r'] % set_size(r') != 0. Non-set items and items of other sets
 * interleave freely; sub-groups may straddle plate boundaries. At the
 * end every set row's count must be a multiple of its set_size.
 */

#include "packingsolver/rectangleguillotine/solution.hpp"

namespace packingsolver
{
namespace rectangleguillotine
{

struct SetsOracleResult
{
    /** No row placed while a sibling row of its set was mid-sub-group. */
    bool order_ok;

    /** Every set row's final count is a multiple of its set_size. */
    bool tail_ok;
};

inline SetsOracleResult check_sets_oracle(
        const Instance& instance,
        const Solution& solution)
{
    std::vector<ItemPos> cnt(instance.number_of_item_types(), 0);
    bool order_ok = true;
    for (BinPos bin_pos = 0;
            bin_pos < solution.number_of_different_bins();
            ++bin_pos) {
        const SolutionBin& bin = solution.bin(bin_pos);
        for (BinPos rep = 0; rep < bin.copies; ++rep) {
            for (const SolutionNode& node: bin.nodes) {
                if (node.d < 1 || node.item_type_id < 0)
                    continue;
                ItemTypeId item_type_id = node.item_type_id;
                const ItemType& item_type = instance.item_type(item_type_id);
                if (item_type.set_id >= 0) {
                    SetId sid = instance.set_id_of_stack(item_type.stack_id);
                    for (StackId s: instance.set_stacks(sid)) {
                        ItemTypeId row_id = instance.item(s, 0);
                        if (row_id == item_type_id)
                            continue;
                        const ItemType& row = instance.item_type(row_id);
                        ItemPos c = cnt[row_id];
                        if (0 < c && c < row.copies && c % row.set_size != 0)
                            order_ok = false;
                    }
                }
                cnt[item_type_id]++;
            }
        }
    }
    bool tail_ok = true;
    for (ItemTypeId item_type_id = 0;
            item_type_id < instance.number_of_item_types();
            ++item_type_id) {
        const ItemType& item_type = instance.item_type(item_type_id);
        if (item_type.set_id >= 0
                && cnt[item_type_id] % item_type.set_size != 0)
            tail_ok = false;
    }
    return {order_ok, tail_ok};
}

}
}
