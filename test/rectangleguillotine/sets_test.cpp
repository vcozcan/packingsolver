#include "packingsolver/rectangleguillotine/instance_builder.hpp"
#include "packingsolver/rectangleguillotine/optimize.hpp"
#include "rectangleguillotine/branching_scheme.hpp"

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

using namespace packingsolver;
using namespace packingsolver::rectangleguillotine;
namespace fs = boost::filesystem;

////////////////////////////////////////////////////////////////////////////////
/////////////////////////// Instance building tests ////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineSets, BasicSetInstance)
{
    // Two item types in set 0 with SET_SIZE=2.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_EQ(instance.number_of_sets(), 1);
    EXPECT_EQ(instance.number_of_items(), 8);
    EXPECT_EQ(instance.item_type(0).set_id, 0);
    EXPECT_EQ(instance.item_type(0).set_size, 2);
    EXPECT_EQ(instance.item_type(1).set_id, 0);
    EXPECT_EQ(instance.item_type(1).set_size, 2);
}

TEST(RectangleGuillotineSets, MixedSetAndNonSetItems)
{
    // Two set items + two non-set items.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(800, 300, -1, 3);
    instance_builder.add_item_type(500, 200, -1, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_EQ(instance.number_of_sets(), 1);
    EXPECT_EQ(instance.number_of_items(), 13);
    // Non-set items retain default set_id/set_size.
    EXPECT_EQ(instance.item_type(2).set_id, -1);
    EXPECT_EQ(instance.item_type(2).set_size, -1);
    EXPECT_EQ(instance.item_type(3).set_id, -1);
}

TEST(RectangleGuillotineSets, MultipleIndependentSets)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(1, 2);
    instance_builder.add_item_type(800, 600, -1, 6);
    instance_builder.set_last_item_type_set(1, 3);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_EQ(instance.number_of_sets(), 2);
    EXPECT_EQ(instance.set_stacks(0).size(), 2u);
    EXPECT_EQ(instance.set_stacks(1).size(), 2u);
}

TEST(RectangleGuillotineSets, SparseSetIds)
{
    // Sparse SET_IDs (50123, 99999) should be remapped to dense (0, 1).
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(50123, 2);
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.set_last_item_type_set(50123, 2);
    instance_builder.add_item_type(800, 300, -1, 6);
    instance_builder.set_last_item_type_set(99999, 3);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_EQ(instance.number_of_sets(), 2);
    // Original SET_IDs preserved on ItemType.
    EXPECT_EQ(instance.item_type(0).set_id, 50123);
    EXPECT_EQ(instance.item_type(2).set_id, 99999);
}

TEST(RectangleGuillotineSets, NonSetInstanceHasSetsIsFalse)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_FALSE(instance.has_sets());
    EXPECT_EQ(instance.number_of_sets(), 0);
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Validation tests ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineSets, MutualExclusionSetAndStackSameItem)
{
    // Same item has both explicit stack_id and set_id — rejected.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4, false, 0);  // explicit stack_id=0
    instance_builder.set_last_item_type_set(0, 2);                // also set on same item
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineSets, MixedSetAndExplicitStackBuilds)
{
    // One item with explicit stack_id, another with set_id — allowed.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4, false, 0);  // explicit stack_id=0
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_EQ(instance.number_of_sets(), 1);
    // Stack item: stack_id=0, no set
    EXPECT_EQ(instance.item_type(0).set_id, -1);
    // Set item: auto-assigned stack, set_id=0
    EXPECT_EQ(instance.item_type(1).set_id, 0);
}

TEST(RectangleGuillotineSets, CopiesNotDivisibleBySetSize)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 5);  // 5 not divisible by 2
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineSets, SetSizeWithoutSetId)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);  // valid set item
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.set_last_item_type_set(-1, 2);  // SET_SIZE without SET_ID
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineSets, OrphanSetSizeNoSetAnywhere)
{
    // SET_SIZE set but no SET_ID anywhere in the instance.
    // The unconditional orphan check should catch this.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(-1, 2);
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineSets, NegativeSetId)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(-2, 2);  // negative but not -1
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineSets, MissingSetSize)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, -1);  // SET_ID without SET_SIZE
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineSets, SetSizeZero)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 0);  // SET_SIZE=0 invalid
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// CSV parsing tests /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineSets, CsvBasicSet)
{
    InstanceBuilder instance_builder;
    instance_builder.read_item_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_basic" / "items.csv").string());
    instance_builder.read_bin_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_basic" / "bins.csv").string());
    instance_builder.read_parameters(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_basic" / "parameters.csv").string());
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_EQ(instance.number_of_sets(), 1);
    EXPECT_EQ(instance.number_of_items(), 8);
    EXPECT_EQ(instance.item_type(0).set_id, 0);
    EXPECT_EQ(instance.item_type(0).set_size, 2);
}

TEST(RectangleGuillotineSets, CsvMixedSetAndNonSet)
{
    InstanceBuilder instance_builder;
    instance_builder.read_item_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_mixed" / "items.csv").string());
    instance_builder.read_bin_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_mixed" / "bins.csv").string());
    instance_builder.read_parameters(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_mixed" / "parameters.csv").string());
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_EQ(instance.number_of_items(), 13);
    EXPECT_EQ(instance.item_type(2).set_id, -1);
}

TEST(RectangleGuillotineSets, CsvMultipleSets)
{
    InstanceBuilder instance_builder;
    instance_builder.read_item_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_multiple" / "items.csv").string());
    instance_builder.read_bin_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_multiple" / "bins.csv").string());
    instance_builder.read_parameters(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_multiple" / "parameters.csv").string());
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_EQ(instance.number_of_sets(), 2);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////// Branching scheme tests //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineSets, StackPredCrossSetBroken)
{
    // Three identical rows across two sets.
    // stack_pred_ must not link stacks across set boundaries.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_set(1, 2);
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    BranchingScheme branching_scheme(instance);
    // Should build without error. The cross-set links should be broken.
    auto root = branching_scheme.root();
    EXPECT_NE(root, nullptr);
}

TEST(RectangleGuillotineSets, StackPredDifferentSetSizeBroken)
{
    // Two rows with same geometry/copies but different SET_SIZE.
    // stack_pred_ must not link them.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 6);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(1000, 500, -1, 6);
    instance_builder.set_last_item_type_set(0, 3);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    BranchingScheme branching_scheme(instance);
    auto root = branching_scheme.root();
    EXPECT_NE(root, nullptr);
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////// Algorithm selection tests ////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineSets, TreeSearchForcedForSets)
{
    // Sets instance should run tree search and complete without error.
    InstanceBuilder instance_builder;
    instance_builder.read_item_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_basic" / "items.csv").string());
    instance_builder.read_bin_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_basic" / "bins.csv").string());
    instance_builder.read_parameters(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_basic" / "parameters.csv").string());
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    auto output = optimize(instance, optimize_parameters);

    // All 8 items should be placed.
    EXPECT_EQ(output.solution_pool.best().number_of_items(), 8);
}

TEST(RectangleGuillotineSets, ExplicitIncompatibleAlgorithmRejected)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    optimize_parameters.use_sequential_value_correction = true;

    EXPECT_THROW(optimize(instance, optimize_parameters),
                 std::invalid_argument);
}

TEST(RectangleGuillotineSets, SingleRowSetSolves)
{
    // Single row with 20 copies, SET_SIZE=2.
    // Items are 1000x500 each (10M total), bin is 6000x3210 (19.26M).
    InstanceBuilder instance_builder;
    instance_builder.read_item_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_single_row" / "items.csv").string());
    instance_builder.read_bin_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_single_row" / "bins.csv").string());
    instance_builder.read_parameters(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_single_row" / "parameters.csv").string());
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    auto output = optimize(instance, optimize_parameters);

    EXPECT_EQ(output.solution_pool.best().number_of_items(), 20);
}

TEST(RectangleGuillotineSets, MixedSetNonSetSolves)
{
    InstanceBuilder instance_builder;
    instance_builder.read_item_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_mixed" / "items.csv").string());
    instance_builder.read_bin_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_mixed" / "bins.csv").string());
    instance_builder.read_parameters(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_mixed" / "parameters.csv").string());
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    auto output = optimize(instance, optimize_parameters);

    // All 13 items (8 set + 5 non-set) should be placed.
    EXPECT_EQ(output.solution_pool.best().number_of_items(), 13);
}

TEST(RectangleGuillotineSets, MultipleSetssolves)
{
    InstanceBuilder instance_builder;
    instance_builder.read_item_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_multiple" / "items.csv").string());
    instance_builder.read_bin_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_multiple" / "bins.csv").string());
    instance_builder.read_parameters(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_multiple" / "parameters.csv").string());
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    auto output = optimize(instance, optimize_parameters);

    // All 18 items should be placed.
    EXPECT_EQ(output.solution_pool.best().number_of_items(), 18);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////// same_plate_sets option tests //////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Count the number of physical plates (bins, counting bin copies) that contain
// at least one item belonging to set 'set_id' (the original, user-provided
// SET_ID preserved on ItemType). When same_plate_sets is enabled this must be
// exactly 1 for every set; the assertion is invariant to how the rest of the
// solution is arranged, so it stays robust under Anytime nondeterminism.
static int plates_with_set(const Solution& solution, SetId set_id)
{
    const Instance& instance = solution.instance();
    int plates = 0;
    for (BinPos bin_pos = 0;
            bin_pos < solution.number_of_different_bins();
            ++bin_pos) {
        const SolutionBin& bin = solution.bin(bin_pos);
        for (const SolutionNode& node: bin.nodes) {
            if (node.f != -1
                    && node.item_type_id >= 0
                    && instance.item_type(node.item_type_id).set_id == set_id) {
                plates += bin.copies;
                break;
            }
        }
    }
    return plates;
}

TEST(RectangleGuillotineSets, SamePlateSetsDefaultOffNonSetRegression)
{
    // Non-set instance: the same_plate_sets code path must be completely inert
    // when OFF (the default), so behavior is unchanged.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.add_bin_type(2000, 1000, -1, 2);
    Instance instance = instance_builder.build();

    EXPECT_FALSE(instance.parameters().same_plate_sets);

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    auto output = optimize(instance, optimize_parameters);

    EXPECT_EQ(output.solution_pool.best().number_of_items(), 4);
}

TEST(RectangleGuillotineSets, SamePlateSetsDefaultOffSetUnchanged)
{
    // A set that fits on one bin, solved with the flag OFF: the set path is
    // unaffected by the new parameter and all items are placed.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(2000, 1000, -1, 2);
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_FALSE(instance.parameters().same_plate_sets);

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    auto output = optimize(instance, optimize_parameters);

    EXPECT_EQ(output.solution_pool.best().number_of_items(), 4);
}

TEST(RectangleGuillotineSets, SamePlateSetsKeepsSetTogether)
{
    // Set (2 copies, fits one bin) + 3 non-set fillers; bins hold 2 pieces, so
    // 3 bins are used. Without the flag the set could scatter; with it ON every
    // set member must share a single plate. Anytime (the production mode) is
    // used so the veto can redirect rather than dead-end.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 1000, -1, 2);  // set row, 2 copies
    instance_builder.set_last_item_type_set(0, 1);
    instance_builder.add_item_type(1000, 1000, -1, 3);  // non-set filler
    instance_builder.add_bin_type(2000, 1000, -1, 3);
    instance_builder.set_same_plate_sets(true);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::Anytime;
    optimize_parameters.verbosity_level = 0;
    optimize_parameters.timer.set_time_limit(2.0);
    auto output = optimize(instance, optimize_parameters);
    const Solution& solution = output.solution_pool.best();

    EXPECT_EQ(solution.number_of_items(), 5);   // all placed (no dead-end)
    EXPECT_EQ(plates_with_set(solution, 0), 1);  // set kept on one plate
}

TEST(RectangleGuillotineSets, SamePlateSetsWholeSetMultiRow)
{
    // The case the active-row (% SET_SIZE) rule misses: a 2-row set with
    // SET_SIZE=2 and COPIES=2 per row. After both copies of row A are placed
    // (pos_stack[A]=2, 2 % 2 == 0 -> active-row rule does NOT fire), row B is
    // still unplaced; only the WHOLE-SET veto keeps the set on one plate.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 2);  // set row A
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(1000, 500, -1, 2);  // set row B
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(1000, 500, -1, 2);  // non-set filler
    instance_builder.add_bin_type(2000, 1000, -1, 3);
    instance_builder.set_same_plate_sets(true);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::Anytime;
    optimize_parameters.verbosity_level = 0;
    optimize_parameters.timer.set_time_limit(2.0);
    auto output = optimize(instance, optimize_parameters);
    const Solution& solution = output.solution_pool.best();

    EXPECT_EQ(solution.number_of_items(), 6);   // all 4 set + 2 filler placed
    EXPECT_EQ(plates_with_set(solution, 0), 1);  // whole set on one plate
}

TEST(RectangleGuillotineSets, SamePlateSetsMultipleSetsEachOnePlate)
{
    // Two independent single-row sets; each must land entirely on its own plate.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 1000, -1, 2);
    instance_builder.set_last_item_type_set(0, 1);
    instance_builder.add_item_type(1000, 1000, -1, 2);
    instance_builder.set_last_item_type_set(1, 1);
    instance_builder.add_bin_type(2000, 1000, -1, 3);
    instance_builder.set_same_plate_sets(true);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::Anytime;
    optimize_parameters.verbosity_level = 0;
    optimize_parameters.timer.set_time_limit(2.0);
    auto output = optimize(instance, optimize_parameters);
    const Solution& solution = output.solution_pool.best();

    EXPECT_EQ(solution.number_of_items(), 4);
    EXPECT_EQ(plates_with_set(solution, 0), 1);
    EXPECT_EQ(plates_with_set(solution, 1), 1);
}

TEST(RectangleGuillotineSets, SamePlateSetsOversizedSetBites)
{
    // A single-row set of 3 copies of 1000x1000; each 2000x1000 bin holds only
    // 2, so the whole set cannot fit on one plate. Deterministic greedy
    // (NotAnytimeSequential) makes the contrast crisp:
    //   - OFF: all 3 placed, the set spans 2 plates.
    //   - ON:  the set cannot open a second bin while partial, so it is left
    //          incomplete and what is placed stays on a single plate.
    auto build = [](bool same_plate_sets) {
        InstanceBuilder instance_builder;
        instance_builder.set_objective(Objective::BinPackingWithLeftovers);
        instance_builder.set_number_of_stages(3);
        instance_builder.set_cut_type(CutType::NonExact);
        instance_builder.add_item_type(1000, 1000, -1, 3);
        instance_builder.set_last_item_type_set(0, 1);
        instance_builder.add_bin_type(2000, 1000, -1, 2);
        instance_builder.set_same_plate_sets(same_plate_sets);
        return instance_builder.build();
    };

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;

    Instance instance_off = build(false);
    auto output_off = optimize(instance_off, optimize_parameters);
    const Solution& solution_off = output_off.solution_pool.best();
    EXPECT_EQ(solution_off.number_of_items(), 3);
    EXPECT_GE(plates_with_set(solution_off, 0), 2);  // OFF splits the set

    Instance instance_on = build(true);
    auto output_on = optimize(instance_on, optimize_parameters);
    const Solution& solution_on = output_on.solution_pool.best();
    EXPECT_LT(solution_on.number_of_items(), 3);      // ON cannot complete it
    EXPECT_EQ(plates_with_set(solution_on, 0), 1);    // partial set stays put
}
