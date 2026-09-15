#include "rectangleguillotine/branching_scheme.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

using namespace packingsolver;
using namespace packingsolver::rectangleguillotine;

namespace
{

using Node = BranchingScheme::Node;

struct NodeLifetimeTracker
{
    std::vector<int> deletions;
    int deletion_depth = 0;
    int maximum_deletion_depth = 0;

    std::shared_ptr<Node> make_node(std::shared_ptr<Node> parent = nullptr)
    {
        const auto id = deletions.size();
        deletions.push_back(0);
        auto node = std::shared_ptr<Node>(new Node(), [this, id](Node* node)
                {
                    ++deletion_depth;
                    maximum_deletion_depth = std::max(
                            maximum_deletion_depth, deletion_depth);
                    delete node;
                    --deletion_depth;
                    ++deletions[id];
                });
        node->id = id;
        node->parent = std::move(parent);
        return node;
    }
};

}

TEST(RectangleGuillotineNodeLifetime, DeepChainHasBoundedDeletionDepth)
{
    NodeLifetimeTracker tracker;
    std::shared_ptr<Node> leaf;
    for (int i = 0; i < 100000; ++i)
        leaf = tracker.make_node(std::move(leaf));

    leaf.reset();

    EXPECT_LE(tracker.maximum_deletion_depth, 2);
    EXPECT_TRUE(std::all_of(tracker.deletions.begin(), tracker.deletions.end(),
                [](int count) { return count == 1; }));
}

TEST(RectangleGuillotineNodeLifetime, DeepMakeSharedChainReleasesEveryNode)
{
    std::vector<std::weak_ptr<Node>> nodes;
    std::shared_ptr<Node> leaf;
    for (int i = 0; i < 100000; ++i) {
        auto node = std::make_shared<Node>();
        node->parent = std::move(leaf);
        nodes.push_back(node);
        leaf = std::move(node);
    }

    leaf.reset();

    EXPECT_TRUE(std::all_of(nodes.begin(), nodes.end(),
                [](const std::weak_ptr<Node>& node) { return node.expired(); }));
}

TEST(RectangleGuillotineNodeLifetime, SharedAncestorSurvivesBranchRelease)
{
    NodeLifetimeTracker tracker;
    std::shared_ptr<Node> ancestor;
    const int ancestor_count = 40000;
    const int branch_length = 30000;
    for (int i = 0; i < ancestor_count; ++i)
        ancestor = tracker.make_node(std::move(ancestor));
    ancestor->pos_stack = {3, 7};

    auto first = ancestor;
    auto second = ancestor;
    ancestor.reset();
    for (int i = 0; i < branch_length; ++i)
        first = tracker.make_node(std::move(first));
    for (int i = 0; i < branch_length; ++i)
        second = tracker.make_node(std::move(second));

    first.reset();

    for (int i = 0; i < static_cast<int>(tracker.deletions.size()); ++i) {
        const bool released_branch = i >= ancestor_count
            && i < ancestor_count + branch_length;
        ASSERT_EQ(tracker.deletions[i], released_branch? 1: 0) << i;
    }
    // Traverse the surviving branch all the way to the root. Cleanup must not
    // detach the shared ancestor or alter its search state.
    auto current = second;
    for (int i = 0; i < branch_length; ++i) {
        ASSERT_TRUE(current != nullptr);
        EXPECT_EQ(current->id, ancestor_count + 2 * branch_length - 1 - i);
        current = current->parent;
    }
    ASSERT_TRUE(current != nullptr);
    EXPECT_EQ(current->pos_stack, (std::vector<ItemPos>{3, 7}));
    for (int i = ancestor_count - 1; i >= 0; --i) {
        ASSERT_TRUE(current != nullptr);
        EXPECT_EQ(current->id, i);
        current = current->parent;
    }
    EXPECT_TRUE(current == nullptr);

    second.reset();

    EXPECT_LE(tracker.maximum_deletion_depth, 2);
    EXPECT_TRUE(std::all_of(tracker.deletions.begin(), tracker.deletions.end(),
                [](int count) { return count == 1; }));
}

TEST(RectangleGuillotineNodeLifetime, CopyAndMovePreserveParentOwnership)
{
    auto parent = std::make_shared<Node>();
    parent->id = 42;
    Node node;
    node.parent = parent;
    node.pos_stack = {3, 7};

    Node copy(node);
    EXPECT_TRUE(copy.parent == parent);
    EXPECT_EQ(copy.pos_stack, node.pos_stack);

    Node moved(std::move(node));
    EXPECT_TRUE(node.parent == nullptr);
    EXPECT_TRUE(moved.parent == parent);
    EXPECT_EQ(moved.pos_stack, copy.pos_stack);

    node = copy;
    EXPECT_TRUE(node.parent == parent);
    moved = std::move(node);
    EXPECT_TRUE(node.parent == nullptr);
    EXPECT_TRUE(moved.parent == parent);
    EXPECT_EQ(parent->id, 42);
}
