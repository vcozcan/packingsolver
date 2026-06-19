#include "packingsolver/rectangleguillotine/solution.hpp"

#include "optimizationtools/utils/utils.hpp"

#include <fstream>
#include <map>

using namespace packingsolver;
using namespace packingsolver::rectangleguillotine;

namespace
{

/// Minimum-waste-length check for ONE axis of a node, accounting for soft trims
/// so that validation agrees with what the search legitimately produces (the
/// search applies the same trim-adjacent discount in branching_scheme.cpp):
///  - The reserved soft-trim BAND (abuts the border at coordinate 0 AND ends at
///    trim - cut_thickness, the exact width the builder lays down) is reserved
///    border the user asked for, NOT a cut-waste sliver the rule guards against
///    -> always satisfied. Otherwise the band (width < min_waste) would wrongly
///    flag the whole solution infeasible (Issue #1, breaking distance > trim).
///  - Usable waste ABUTTING a soft trim (its near edge on the trim line) shares
///    one contiguous breakable border with the trim -> discount the threshold
///    via minimum_waste_length_adjacent_to_soft_trim, the shared source of truth
///    the search uses too (so the two can never silently drift apart).
/// Both relaxations are gated on is_waste_node (item_type_id == -1): a -2 cut
/// separator or -3 residual node that happens to land on these coordinates stays
/// under the full check. The band exemption additionally keys on the band's far
/// edge (not coordinate 0 alone), so a non-band sub-min_waste border waste also
/// stays under the full check on the SolutionBuilder::read / append paths
/// (optimize() only ever emits the band there). The x and y axes differ only by
/// which coordinates/trim are passed, so they share this code — the copy-paste
/// is exactly how the swapped-axis bug class (Issue #2) recurs.
bool axis_min_waste_satisfied(
        bool is_waste_node,
        TrimType trim_type,
        Length trim,
        Length near_coord,
        Length far_coord,
        Length min_waste,
        Length cut_thickness)
{
    Length extent = far_coord - near_coord;
    if (is_waste_node && trim_type == TrimType::Soft && trim > 0) {
        if (near_coord == 0 && far_coord == trim - cut_thickness)
            return true;  // reserved trim band
        if (near_coord == trim)  // usable waste abutting the soft trim
            return extent >= minimum_waste_length_adjacent_to_soft_trim(
                    min_waste, trim);
    }
    return extent >= min_waste;
}

}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// Node /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

std::ostream& print(
        std::ostream& os,
        const std::vector<SolutionNode>& res,
        SolutionNodeId id,
        std::string tab)
{
    os << tab << res[id] << std::endl;
    for (SolutionNodeId c: res[id].children)
        print(os, res, c, tab + "  ");
    return os;
}

std::ostream& packingsolver::rectangleguillotine::operator<<(
        std::ostream& os,
        const SolutionNode& node)
{
    os
        << " f " << node.f
        << " d " << node.d
        << " l " << node.l
        << " r " << node.r
        << " b " << node.b
        << " t " << node.t
        << " item_type_id " << node.item_type_id;
    return os;
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Solution ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Solution::update_indicators(
        BinPos bin_pos)
{
    const SolutionBin& bin = bins_[bin_pos];
    const BinType& bin_type = instance().bin_type(bin.bin_type_id);

    number_of_bins_ += bin.copies;
    bin_copies_[bin.bin_type_id] += bin.copies;
    cost_ += bin.copies * bin_type.cost;
    full_area_ += bin.copies * bin_type.area();
    area_ = full_area_;
    second_leftover_value_ = 0;

    width_ = 0;
    height_ = 0;
    Counter subplate1curr_number_of_2_cuts = 0;
    Length subpalte1curr_end = -1;
    for (const SolutionNode& node: bin.nodes) {
        if (node.f != -1 && node.item_type_id >= 0) {
            number_of_items_ += bin.copies;
            item_area_ += bin.copies * instance().item_type(node.item_type_id).area();
            profit_ += bin.copies * instance().item_type(node.item_type_id).profit;
            item_copies_[node.item_type_id] += bin.copies;
        }

        // Subtract residual area.
        if (node.item_type_id == -3)
            area_ -= (node.t - node.b) * (node.r - node.l);

        // Update width_ and height_.
        if (node.d > 0 && node.item_type_id != -3) {
            if (width_ < node.r)
                width_ = node.r;
            if (height_ < node.t)
                height_ = node.t;
        }

        // Update second_leftover_value_.
        if (node.d == 1 && node.item_type_id != -3)
            second_leftover_value_ = 0;
        if (node.d == 2 && node.item_type_id != -1)
            second_leftover_value_ = 0;
        if (node.d == 2 && node.item_type_id == -1)
            second_leftover_value_ = (node.r - node.l) * (node.t - node.b);

        // Check minimum waste length (per-bin override resolved via effective_*).
        // Per-axis, with soft-trim band exemption + trim-adjacent discount folded
        // into axis_min_waste_satisfied (see its doc). Hard trims reconstruct at
        // d==-1 and never reach this d>=1 check; right/top soft trims are absorbed
        // into the trailing leftover and never emitted as bands.
        if (node.d >= 1
                && node.item_type_id < 0) {
            Length min_waste = effective_minimum_waste_length(
                    bin_type, instance().parameters());
            Length cut_thickness = instance().parameters().cut_thickness;
            // Trims are read raw: there is no per-bin effective_* trim override
            // today (unlike minimum_waste_length above). If one is ever added,
            // route both calls through it for symmetry.
            bool is_waste_node = (node.item_type_id == -1);
            if (!axis_min_waste_satisfied(
                        is_waste_node,
                        bin_type.left_trim_type, bin_type.left_trim,
                        node.l, node.r, min_waste, cut_thickness)
                    || !axis_min_waste_satisfied(
                        is_waste_node,
                        bin_type.bottom_trim_type, bin_type.bottom_trim,
                        node.b, node.t, min_waste, cut_thickness)) {
                minimum_waste_length_feasible_ = false;
                feasible_ = false;
            }
        }

        // Check minimum distance between 1-cuts (per-bin override resolved via effective_*).
        if (node.d == 1
                && node.item_type_id == -2) {
            Length min_dist_1 = effective_minimum_distance_1_cuts(
                    bin_type, instance().parameters());
            if ((bin.first_cut_orientation == CutOrientation::Vertical
                        && node.r - node.l < min_dist_1)
                    || (bin.first_cut_orientation == CutOrientation::Horizontal
                        && node.t - node.b < min_dist_1)) {
                //std::cout << "minimum_distance_1_cuts = false" << std::endl;
                minimum_distance_1_cuts_feasible_ = false;
                feasible_ = false;
            }
        }

        // Check maximum distance between 1-cuts.
        if (instance().parameters().maximum_distance_1_cuts >= 0) {
            if (node.d == 1
                    && node.item_type_id == -2) {
                if ((bin.first_cut_orientation == CutOrientation::Vertical
                            && node.r - node.l
                            > instance().parameters().maximum_distance_1_cuts)
                        || (bin.first_cut_orientation == CutOrientation::Horizontal
                            && node.t - node.b
                            > instance().parameters().maximum_distance_1_cuts)) {
                    //std::cout << "maximum_distance_1_cuts = false" << std::endl;
                    maximum_distance_1_cuts_feasible_ = false;
                    feasible_ = false;
                }
            }
        }

        // Check minimum distance between 2-cuts (per-bin override resolved via effective_*).
        if (node.d == 2
                && node.item_type_id == -2) {
            Length min_dist_2 = effective_minimum_distance_2_cuts(
                    bin_type, instance().parameters());
            if ((bin.first_cut_orientation == CutOrientation::Vertical
                        && node.t - node.b < min_dist_2)
                    || (bin.first_cut_orientation == CutOrientation::Horizontal
                        && node.r - node.l < min_dist_2)) {
                //std::cout << "minimum_distance_2_cuts = false" << std::endl;
                minimum_distance_2_cuts_feasible_ = false;
                feasible_ = false;
            }
        }

        // Check maximum number of 2-cuts.
        if (instance().parameters().maximum_number_2_cuts >= 0) {
            if (node.d == 1) {
                subpalte1curr_end = (bin.first_cut_orientation == CutOrientation::Vertical)?
                    node.t:
                    node.r;
                subplate1curr_number_of_2_cuts = 0;
            }
            if (node.d == 2) {
                if ((bin.first_cut_orientation == CutOrientation::Vertical
                            && node.t != subpalte1curr_end)
                        || (bin.first_cut_orientation == CutOrientation::Horizontal
                            && node.r != subpalte1curr_end)) {
                    subplate1curr_number_of_2_cuts++;
                    if (subplate1curr_number_of_2_cuts
                            > instance().parameters().maximum_number_2_cuts) {
                        //std::cout << "maximum_number_2_cuts = false" << std::endl;
                        maximum_number_2_cuts_feasible_ = false;
                        feasible_ = false;
                    }
                }
            }
        }

        // Check stacks.
        if (node.d >= 1
                && node.item_type_id >= 0) {
            const ItemType& item_type = instance().item_type(node.item_type_id);
            if (item_type.stack_pos > 0) {
                ItemTypeId item_type_id_pred = instance().item(
                        item_type.stack_id,
                        item_type.stack_pos - 1);
                const ItemType& item_type_pred = instance().item_type(item_type_id_pred);
                if (item_copies(item_type_id_pred) != item_type_pred.copies) {
                    //std::cout << "stacks_feasible = false" << std::endl;
                    //std::cout << "item_type_id " << node.item_type_id
                    //    << " stack_id " << item_type.stack_id
                    //    << " stack_pos " << item_type.stack_pos
                    //    << std::endl;
                    //std::cout << "item_type_id_pred " << item_type_id_pred
                    //    << " stack_id " << item_type_pred.stack_id
                    //    << " stack_pos " << item_type_pred.stack_pos
                    //    << " copies " << item_copies(item_type_id_pred)
                    //    << " / " << item_type_pred.copies
                    //    << std::endl;
                    stacks_feasible_ = false;
                    feasible_ = false;
                }
            }
        }

        // Check defect intersections.
        if (node.d >= 1
                && node.item_type_id >= 0) {
            DefectId k = instance().rect_intersects_defect(
                    node.l,
                    node.r,
                    node.b,
                    node.t,
                    bin_type);
            if (k != -1) {
                //std::cout << "defects_feasible = false" << std::endl;
                defects_feasible_ = false;
                feasible_ = false;
            }
        }

        // Check cuts through defects.
        if (!instance().parameters().cut_through_defects
                && node.d >= 1) {
            DefectId kl = instance().x_intersects_defect(
                    node.l,
                    node.b,
                    node.t,
                    bin_type);
            DefectId kr = instance().x_intersects_defect(
                    node.r,
                    node.b,
                    node.t,
                    bin_type);
            DefectId kb = instance().y_intersects_defect(
                    node.l,
                    node.r,
                    node.b,
                    bin_type);
            DefectId kt = instance().y_intersects_defect(
                    node.l,
                    node.r,
                    node.t,
                    bin_type);
            if (kl != -1 || kr != -1 || kb != -1 || kt != -1) {
                cut_through_defects_feasible_ = false;
                feasible_ = false;
            }
        }
    }

    // Check sets: replay the bin's item nodes in physical cut order
    // (a bin with copies k is k sequential plates — deliberately
    // different from the simultaneous-copies semantics of the stacks
    // check above). State persists across bins because sub-groups may
    // legally straddle bin boundaries. Trailing-incomplete runs do NOT
    // flip feasible_ — bins arrive incrementally and a straddling
    // sub-group would transiently look incomplete; strict end-state is
    // exposed via sets_complete(). Flags only — no exit(1).
    if (instance().has_sets()) {
        // Each replay pass is a deterministic function of the entering
        // per-set state (violations included — the resync keeps the
        // evolution deterministic), so once an entering state repeats,
        // the remaining passes replicate already-checked behavior with
        // a fixed period. Jump straight to the final entering state:
        // this keeps high-copy pattern bins O(nodes x distinct states)
        // and subsumes the idle-in/idle-out case (period 1). The
        // canonical mid-run pattern (odd count of one row straddling
        // into the next plate) has period 2, which a plain fixed-point
        // check would miss.
        std::vector<std::vector<ActiveSetState>> entering_states;
        // Indexed lookup: a linear scan over prior passes would be
        // quadratic in bin.copies when the composite state period is
        // large (co-prime per-set periods compound via their LCM and
        // can exceed bin.copies, so no cycle ever fires).
        std::map<std::vector<ActiveSetState>, BinPos> entering_state_to_pass;
        for (BinPos copy = 0; copy < bin.copies; ++copy) {
            auto cycle_it = entering_state_to_pass.find(set_active_states_);
            if (cycle_it != entering_state_to_pass.end()) {
                BinPos cycle_start = cycle_it->second;
                BinPos period = copy - cycle_start;
                BinPos final_pass = cycle_start
                    + (bin.copies - cycle_start) % period;
                set_active_states_ = entering_states[final_pass];
                break;
            }
            entering_state_to_pass.emplace(set_active_states_, copy);
            entering_states.push_back(set_active_states_);
            for (const SolutionNode& node: bin.nodes) {
                if (node.d < 1 || node.item_type_id < 0)
                    continue;
                const ItemType& item_type
                        = instance().item_type(node.item_type_id);
                if (item_type.set_id < 0)
                    continue;
                // ItemType::set_id keeps the original (possibly
                // sparse) CSV value; the dense id lives on the stack.
                SetId sid = instance().set_id_of_stack(item_type.stack_id);
                ActiveSetState& state = set_active_states_[sid];
                if (state.item_type_id != -1
                        && state.item_type_id != node.item_type_id) {
                    // Another row of the same set is mid-sub-group.
                    sets_feasible_ = false;
                    feasible_ = false;
                    // Resync on the new row so later checks stay
                    // meaningful.
                    state = ActiveSetState();
                }
                if (state.item_type_id == -1) {
                    state.item_type_id = node.item_type_id;
                    state.run_count = 0;
                }
                state.run_count++;
                if (state.run_count >= item_type.set_size) {
                    state = ActiveSetState();
                }
            }
        }
    }

    // Check buddies: every buddy group must be co-located on a single
    // physical plate. Record the bins each group's items appear in and
    // flag any group that spans more than one bin entry, or lands in a
    // replicated (copies > 1) pattern bin (which would scatter its pieces
    // across 'copies' physical plates). State persists across calls
    // because bins arrive incrementally; the end-state partial-group
    // check lives in buddies_feasible(). This is independent defense-in-
    // depth: it also catches SolutionBuilder / future-algorithm output
    // that the branching-scheme new-bin guard never saw. Flags only.
    //
    // CONTRACT: buddy_bins_ accumulates across update_indicators() calls and
    // is never reset, which is correct for the single forward pass that
    // builds a solution one bin at a time. It must NOT be fed an append-based
    // merge of buddy sub-solutions: appending one group's pieces under two
    // different bin_pos values would set size() > 1 and flip feasible_
    // irrecoverably. Today no such path exists (CG/SSK/SVC/DS are gated off
    // for buddies in optimize.cpp; tree search uses SolutionBuilder); any
    // future non-TS buddy parity must respect this.
    if (instance().has_buddies()) {
        for (const SolutionNode& node: bin.nodes) {
            if (node.d < 1 || node.item_type_id < 0)
                continue;
            const ItemType& item_type
                    = instance().item_type(node.item_type_id);
            if (item_type.buddy_id < 0)
                continue;
            // ItemType::buddy_id keeps the original (possibly sparse) CSV
            // value; the dense id lives on the stack.
            BuddyId g = instance().buddy_id_of_stack(item_type.stack_id);
            // Defensive: g is in [0, number_of_buddies()) by construction (a
            // buddy item's stack carries a valid dense id). Guard anyway so an
            // inconsistent/partially-built state can never index buddy_bins_
            // out of bounds — flag infeasible and skip instead.
            if (g < 0 || g >= (BuddyId)buddy_bins_.size()) {
                buddies_feasible_ = false;
                feasible_ = false;
                continue;
            }
            buddy_bins_[g].insert(bin_pos);
            if (buddy_bins_[g].size() > 1 || bin.copies > 1) {
                buddies_feasible_ = false;
                feasible_ = false;
            }
        }
    }

    // NOTE: the previous code did exit(1) here, which a library must never do.
    // minimum_waste_length_feasible_ / feasible_ are set above; how an infeasible
    // solution is prevented from being RETURNED depends on the path that built it:
    //  - tree-search reconstruction (BranchingScheme::to_solution) re-checks the
    //    flag and throws std::logic_error (an invariant tripwire that unwinds
    //    without terminating the process);
    //  - append-assembled solutions (column generation / sequential value
    //    correction / single knapsack / dichotomic search build via
    //    SolutionBuilder::build() + Solution::append() and never call
    //    to_solution): no throw fires, but Solution::operator< gates on
    //    feasible(), so an infeasible candidate is ranked worst and dropped by
    //    the SolutionPool (which always retains the feasible empty seed). Worst
    //    case there is a silently-omitted partial result, never a returned
    //    infeasible certificate.
    // With the soft-trim handling above, valid jobs no longer trip the flag, so
    // neither guard fires in normal operation. NOTE for callers: a GENUINELY
    // infeasible non-empty instance (no feasible packing exists at all) therefore
    // surfaces on the append path as best() == the empty seed (0 bins) with a
    // clean exit, NOT as an error. If you need infeasibility signalled, treat
    // "0 bins for a non-empty instance" as infeasible at the call site.
}

bool Solution::sets_complete() const
{
    for (ItemTypeId item_type_id = 0;
            item_type_id < instance().number_of_item_types();
            ++item_type_id) {
        const ItemType& item_type = instance().item_type(item_type_id);
        if (item_type.set_id < 0)
            continue;
        if (item_copies_[item_type_id] % item_type.set_size != 0)
            return false;
    }
    return true;
}

bool Solution::buddies_feasible() const
{
    // Incremental violations (spans > 1 bin, replicated host) were flagged
    // during update_indicators().
    if (!buddies_feasible_)
        return false;
    if (!instance().has_buddies())
        return true;
    // End-state completeness: a buddy group must be fully placed
    // (all copies co-located) or entirely absent — never partial. placed
    // is the total copies of the group's member item types. Each buddy
    // stack is a singleton stack of one item type, so summing item_copies_
    // over the group's member item types double-counts nothing.
    for (BuddyId g = 0; g < instance().number_of_buddies(); ++g) {
        ItemPos placed = 0;
        for (StackId s: instance().buddy_stacks(g))
            placed += item_copies_[instance().item(s, 0)];
        if (placed != 0 && placed != instance().buddy_total(g))
            return false;
    }
    return true;
}

void Solution::append(
        const Solution& solution,
        BinPos bin_pos,
        BinPos copies,
        const std::vector<BinTypeId>& bin_type_ids,
        const std::vector<ItemTypeId>& item_type_ids)
{
    if (number_of_different_bins() > 0) {
        SolutionNode& node = bins_.back().nodes.back();
        if (node.item_type_id == -3) {
            node.item_type_id = -1;
            area_ -= (node.t - node.b) * (node.r - node.l);
        }
    }
    const SolutionBin& bin_old = solution.bin(bin_pos);
    BinTypeId bin_type_id = (bin_type_ids.empty())?
        bin_old.bin_type_id:
        bin_type_ids[bin_old.bin_type_id];
    SolutionBin bin;
    bin.bin_type_id = bin_type_id;
    bin.copies = copies;
    bin.first_cut_orientation = bin_old.first_cut_orientation;
    for (SolutionNode node: bin_old.nodes) {
        if (node.t == -4) {
        } else if (node.f == -1) {
            node.item_type_id = bin_type_id;
        } else if (node.item_type_id >= 0) {
            node.item_type_id = (item_type_ids.empty())?
                node.item_type_id:
                item_type_ids[node.item_type_id];
        }
        bin.nodes.push_back(node);
    }
    bins_.push_back(bin);
    update_indicators(bins_.size() - 1);
}

void Solution::append(
        const Solution& solution,
        const std::vector<BinTypeId>& bin_type_ids,
        const std::vector<ItemTypeId>& item_type_ids)
{
    for (BinPos bin_pos = 0; bin_pos < solution.number_of_bins(); ++bin_pos)
        append(solution, bin_pos, 1, bin_type_ids, item_type_ids);
}

bool Solution::operator<(const Solution& solution) const
{
    // Check feasibility. feasible() folds in end-state buddy completeness: a
    // partial buddy group is infeasible even when the per-bin flags never
    // tripped (the group sits on one bin but isn't fully placed), so the raw
    // feasible_ member alone could prefer such a solution.
    if (!solution.feasible())
        return false;
    if (!feasible())
        return true;

    switch (instance().objective()) {
    case Objective::Default: {
        if (solution.profit() < profit())
            return false;
        if (solution.profit() > profit())
            return true;
        return solution.waste() < waste();
    } case Objective::BinPacking: {
        if (!solution.full())
            return false;
        if (!full())
            return true;
        return solution.number_of_bins() < number_of_bins();
    } case Objective::BinPackingWithLeftovers: {
        if (!solution.full())
            return false;
        if (!full())
            return true;
        if (solution.waste() != waste())
            return solution.waste() < waste();
        return solution.second_leftover_value() > second_leftover_value();
    } case Objective::OpenDimensionX: {
        if (!solution.full())
            return false;
        if (!full())
            return true;
        return solution.width() < width();
    } case Objective::OpenDimensionY: {
        if (!solution.full())
            return false;
        if (!full())
            return true;
        return solution.height() < height();
    } case Objective::Knapsack: {
        return strictly_greater(solution.profit(), profit());
    } case Objective::VariableSizedBinPacking: {
        if (!solution.full())
            return false;
        if (!full())
            return true;
        return strictly_lesser(solution.cost(), cost());
    } default: {
        std::stringstream ss;
        ss << FUNC_SIGNATURE << ": "
            << "solution rectangleguillotine::Solution does not support objective \""
            << instance().objective() << "\"";
        throw std::logic_error(ss.str());
    }
    }
}

void Solution::write(
        const std::string& certificate_path) const
{
    if (certificate_path.empty())
        return;
    std::ofstream file{certificate_path};
    if (!file.good()) {
        throw std::runtime_error(
                FUNC_SIGNATURE + ": "
                "unable to open file \"" + certificate_path + "\".");
    }

    file << "PLATE_ID,COPIES,NODE_ID,X,Y,WIDTH,HEIGHT,TYPE,CUT,PARENT" << std::endl;
    SolutionNodeId offset = 0;
    for (BinPos bin_pos = 0; bin_pos < number_of_different_bins(); ++bin_pos) {
        const SolutionBin& solution_bin = bins_[bin_pos];
        BinTypeId bin_type_id = solution_bin.bin_type_id;
        const BinType& bin_type = instance().bin_type(bin_type_id);
        for (SolutionNodeId node_id = 0;
                node_id < (SolutionNodeId)solution_bin.nodes.size();
                ++node_id) {
            const SolutionNode& n = solution_bin.nodes[node_id];
            file
                << bin_pos << ","
                << solution_bin.copies << ","
                << offset + node_id << ","
                << n.l << ","
                << n.b << ","
                << n.r - n.l << ","
                << n.t - n.b << ","
                << n.item_type_id << ","
                << n.d << ",";
            if (n.f != -1)
                file << offset + n.f;
            file << std::endl;
        }
        offset += solution_bin.nodes.size();
        for (const Defect& defect: bin_type.defects) {
            file
                << bin_pos << ","
                << solution_bin.copies << ","
                << -1 << ","
                << defect.pos.x << ","
                << defect.pos.y << ","
                << defect.rect.w << ","
                << defect.rect.h << ","
                << -4 << ","
                << -1 << ","
                << std::endl;
        }
    }
}

nlohmann::json Solution::to_json() const
{
    return nlohmann::json {
        {"NumberOfItems", number_of_items()},
        {"ItemArea", item_area()},
        {"ItemProfit", profit()},
        {"NumberOfBins", number_of_bins()},
        {"NumberOfDifferentBins", number_of_different_bins()},
        {"BinArea", full_area()},
        {"BinCost", cost()},
        {"Waste", waste()},
        {"WastePercentage", waste_percentage()},
        {"FullWaste", full_waste()},
        {"FullWastePercentage", full_waste_percentage()},
        {"Width", width()},
        {"Height", height()},
        {"SecondLeftoverValue", second_leftover_value()},
    };
}

void Solution::format(
        std::ostream& os,
        int verbosity_level) const
{
    if (verbosity_level >= 1) {
        os
            << "Number of items:           " << optimizationtools::Ratio<ItemPos>(number_of_items(), instance().number_of_items()) << std::endl
            << "Item area:                 " << optimizationtools::Ratio<Area>(item_area(), instance().item_area()) << std::endl
            << "Item profit:               " << optimizationtools::Ratio<Profit>(profit(), instance().item_profit()) << std::endl
            << "Number of bins:            " << optimizationtools::Ratio<BinPos>(number_of_bins(), instance().number_of_bins()) << std::endl
            << "Number of different bins:  " << number_of_different_bins() << std::endl
            << "Bin area:                  " << optimizationtools::Ratio<BinPos>(full_area(), instance().bin_area()) << std::endl
            << "Bin cost:                  " << cost() << std::endl
            << "Waste:                     " << waste() << " (" << 100 * waste_percentage() << "%)" << std::endl
            << "Full waste:                " << full_waste() << " (" << 100 * full_waste_percentage() << "%)" << std::endl
            << "Width:                     " << width() << std::endl
            << "Height:                    " << height() << std::endl
            << "Second leftover value:     " << second_leftover_value() << std::endl
            ;
    }

    if (verbosity_level >= 2) {
        os
            << std::right << std::endl
            << std::setw(12) << "Bin"
            << std::setw(12) << "Type"
            << std::setw(12) << "Copies"
            << std::endl
            << std::setw(12) << "---"
            << std::setw(12) << "----"
            << std::setw(12) << "------"
            << std::endl;
        for (BinPos bin_pos = 0;
                bin_pos < number_of_different_bins();
                ++bin_pos) {
            os
                << std::setw(12) << bin_pos
                << std::setw(12) << bin(bin_pos).bin_type_id
                << std::setw(12) << bin(bin_pos).copies
                << std::endl;
        }
    }

    if (verbosity_level >= 3) {
        os
            << std::right << std::endl
            << std::setw(12) << "Bin"
            << std::setw(12) << "Node"
            << std::setw(12) << "Parent"
            << std::setw(12) << "Depth"
            << std::setw(12) << "Left"
            << std::setw(12) << "Right"
            << std::setw(12) << "Bottom"
            << std::setw(12) << "Top"
            << std::setw(12) << "Item"
            << std::endl
            << std::setw(12) << "---"
            << std::setw(12) << "----"
            << std::setw(12) << "------"
            << std::setw(12) << "-----"
            << std::setw(12) << "----"
            << std::setw(12) << "-----"
            << std::setw(12) << "------"
            << std::setw(12) << "---"
            << std::setw(12) << "----"
            << std::endl;
        for (BinPos bin_pos = 0; bin_pos < number_of_different_bins(); ++bin_pos) {
            const SolutionBin& solution_bin = bins_[bin_pos];
            for (SolutionNodeId node_id = 0;
                    node_id < (SolutionNodeId)solution_bin.nodes.size();
                    ++node_id) {
                const SolutionNode& node = solution_bin.nodes[node_id];
                //const BinType& bin_type = instance().bin_type(bin(bin_pos).i);
                os
                    << std::setw(12) << bin_pos
                    << std::setw(12) << node_id
                    << std::setw(12) << node.f
                    << std::setw(12) << node.d
                    << std::setw(12) << node.l
                    << std::setw(12) << node.r
                    << std::setw(12) << node.b
                    << std::setw(12) << node.t
                    << std::setw(12) << node.item_type_id
                    << std::endl;
            }
        }
    }

    if (verbosity_level >= 3) {
        os
            << std::right << std::endl
            << std::setw(12) << "Item type"
            << std::setw(12) << "Width"
            << std::setw(12) << "Height"
            << std::setw(12) << "Copies"
            << std::endl
            << std::setw(12) << "---------"
            << std::setw(12) << "-----"
            << std::setw(12) << "------"
            << std::setw(12) << "------"
            << std::endl;
        for (ItemTypeId item_type_id = 0;
                item_type_id < instance().number_of_item_types();
                ++item_type_id) {
            const ItemType& item_type = instance().item_type(item_type_id);
            os
                << std::setw(12) << item_type_id
                << std::setw(12) << item_type.rect.w
                << std::setw(12) << item_type.rect.h
                << std::setw(12) << item_copies(item_type_id)
                << std::endl;
        }
    }
}
