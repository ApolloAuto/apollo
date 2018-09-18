/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "cybertron/scheduler/policy/rbtree.h"

#define node_parent(n) ((RBNode *)(n->parent_color_ & ~3))
#define node_color(n) (n->parent_color_ & 1)
#define node_is_red(n) (!node_color(n))
#define node_is_black(n) node_color(n)
#define node_gparent(n) \
  ((RBNode *)(((RBNode *)(n->parent_color_ & ~3))->parent_color_ & ~3))
#define node_set_red(n)     \
  do {                      \
    n->parent_color_ &= ~1; \
  } while (0)
#define node_set_black(n)  \
  do {                     \
    n->parent_color_ |= 1; \
  } while (0)
#define node_set_color(n, c)                      \
  do {                                            \
    n->parent_color_ = n->parent_color_ & ~1 | c; \
  } while (0)
#define node_set_parent(p, n)                              \
  do {                                                     \
    n->parent_color_ = (uint64_t)p | n->parent_color_ & 1; \
  } while (0)

namespace apollo {
namespace cybertron {
namespace scheduler {

void RBTree::LinkNode(RBNode **link, RBNode *node, RBNode *parent) {
  *link = node;
  node->parent_color_ = (uint64_t)parent & ~1;
  node->left_ = node->right_ = nullptr;
}

void RBTree::InsertRebalance(RBNode *node) {
  RBNode *parent, *gparent;

  while ((parent = node_parent(node)) && node_is_red(parent)) {
    gparent = node_gparent(node);

    if (parent == gparent->left_) {
      RBNode *uncle = gparent->right_;

      if (uncle && node_is_red(uncle)) {
        node_set_black(parent);
        node_set_black(uncle);
        node_set_red(gparent);

        node = gparent;
      } else {
        if (node == parent->right_) {
          RBNode *tmp;

          RotateLeft(parent);

          tmp = parent;
          parent = node;
          node = tmp;
        }

        node_set_black(parent);
        node_set_red(gparent);
        RotateRight(gparent);
      }
    } else {
      RBNode *uncle = gparent->left_;

      if (uncle && node_is_red(uncle)) {
        node_set_black(parent);
        node_set_black(uncle);
        node_set_red(gparent);

        node = gparent;
      } else {
        if (node == parent->left_) {
          RBNode *tmp;

          RotateRight(parent);

          tmp = parent;
          parent = node;
          node = tmp;
        }

        node_set_black(parent);
        node_set_red(gparent);
        RotateLeft(gparent);
      }
    }
  }

  node_set_black(root_);
}

void RBTree::RotateLeft(RBNode *node) {
  RBNode *tmp = node->right_;

  node_set_parent(node_parent(node), tmp);
  if (node_parent(node) == 0)
    root_ = tmp;
  else if (node == node_parent(node)->left_)
    node_parent(node)->left_ = tmp;
  else
    node_parent(node)->right_ = tmp;

  if (tmp->left_) {
    node_set_parent(node, tmp->left_);
    node->right_ = tmp->left_;
  } else {
    node->right_ = nullptr;
  }

  node_set_parent(tmp, node);
  tmp->left_ = node;
}

void RBTree::RotateRight(RBNode *node) {
  RBNode *tmp = node->left_;

  node_set_parent(node_parent(node), tmp);
  if (node_parent(node) == 0)
    root_ = tmp;
  else if (node == node_parent(node)->left_)
    node_parent(node)->left_ = tmp;
  else
    node_parent(node)->right_ = tmp;

  if (tmp->right_) {
    node_set_parent(node, tmp->right_);
    node->left_ = tmp->right_;
  } else {
    node->left_ = nullptr;
  }

  node_set_parent(tmp, node);
  tmp->right_ = node;
}

void RBTree::Insert(RBNode *node) {
  RBNode **p = &(root_), *parent = nullptr;

  if (*p == nullptr) {
    *p = node;
    node->parent_color_ = (uint64_t) nullptr | 1;
    node->left_ = node->right_ = nullptr;
    return;
  }

  while (*p) {
    RBNode *n = reinterpret_cast<RBNode *>(*p);

    parent = *p;
    p = node->Compare(n) ? &((*p)->right_) : &((*p)->left_);
  }

  LinkNode(p, node, parent);
  InsertRebalance(node);
}

RBNode *RBTree::First() {
  RBNode *n = root_;

  if (!n) return nullptr;

  while (n->left_) n = n->left_;

  return n;
}

void RBTree::Preorder() { Preorder(root_); }

void RBTree::Preorder(RBNode *node) {
  if (node == nullptr) return;

  node->Print();

  Preorder(node->left_);
  Preorder(node->right_);
}

RBNode *RBTree::Find(void *key) {
  RBNode *n = root_;

  while (n) {
    if (n->Compare(key) > 0)
      n = n->left_;
    else if (n->Compare(key) < 0)
      n = n->right_;
    else
      break;
  }

  return n;
}

void RBTree::Delete(RBNode *node) {
  RBNode *parent, *child;
  int color;

  if (node->left_ && node->right_) {
    RBNode *old = node, *left;

    // Use in-order successor as substitution
    node = node->right_;
    while ((left = node->left_)) node = left;

    child = node->right_;
    parent = node_parent(node);
    color = node_color(node);

    // Link node's subtree to node's parent.
    if (child) node_set_parent(parent, child);
    if (parent == old) {
      // Node right child has no left subtree.
      parent->right_ = child;
      // In this case, node will replace old later, then the node will be the
      // parent,
      // so update parent pointer from old to the node.
      parent = node;
    } else {
      parent->left_ = child;
    }

    // Replace old with node
    // Keep old color to keep balance.
    node->parent_color_ = old->parent_color_;
    node->left_ = old->left_;
    node->right_ = old->right_;

    node_set_parent(node, old->left_);
    if (old->right_) {
      // If node(to be del)'s right subtree has only one node, then child will
      // be null, and
      // met "parent == old", so this snippet "parent->right_ = child" will also
      // set old->right_
      // as null indirectly. Here check this case.
      node_set_parent(node, old->right_);
    }

    RBNode *tmp = node_parent(old);
    if (tmp) {
      if (tmp->left_ == old)
        tmp->left_ = node;
      else
        tmp->right_ = node;
    } else {
      node_set_black(node);
      root_ = node;
      return;
    }

    if (color) DeleteRebalance(child, parent);

    return;
  } else if (!node->left_) {
    child = node->right_;
  } else {
    child = node->left_;
  }

  parent = node_parent(node);
  color = node_color(node);

  if (child) node_set_parent(parent, child);

  if (parent) {
    if (parent->left_ == node)
      parent->left_ = child;
    else
      parent->right_ = child;

    if (color) DeleteRebalance(child, parent);
  } else {
    root_ = child;
    if (child) {
      node_set_black(child);
    }
  }
}

void RBTree::DeleteRebalance(RBNode *node, RBNode *parent) {
  RBNode *sibling;

  while ((!node || node_is_black(node)) && node != root_) {
    if (parent->left_ == node) {
      sibling = parent->right_;

      if (node_is_red(sibling)) {
        // Case 1: Sibling is red
        // Recolor parent to red; Recolor sibling to black; Left rotate at
        // parent.
        node_set_red(parent);
        node_set_black(sibling);
        RotateLeft(parent);
        // After left rotate, sibling's parent's right child maybe is changed,
        // so update sibling.
        sibling = parent->right_;
      }

      if ((!sibling->left_ || node_is_black(sibling->left_)) &&
              (!sibling->right_) ||
          node_is_black(sibling->right_)) {
        // Case 2: Sibling is black and its both child ard black
        // Recolor sibling to red; Set parent as node; Set gparent as parent.
        node_set_red(sibling);
        node = parent;
        parent = node_parent(parent);
      } else {
        // Case 3,4: Sibling is black and at least one of sibling’s children is
        // red.

        if (!sibling->right_ || node_is_black(sibling->right_)) {
          // Case 3: Right is black (it means left is red, otherwise enter
          // branch "both black")
          // Recolor sibling to red; Recolor sibling's left to Black; Right
          // rotate at sibling.
          node_set_red(sibling);
          if (sibling->left_) node_set_black(sibling->left_);

          RotateRight(sibling);
          // Ather right rotate, sibling's parent's right child maybe is
          // changed, so udpate sibling.
          sibling = parent->right_;
        }

        // Case 4: Right is red, left is any.
        // Recolor sibling same as parent; Recolor parent to black; Recolor
        // sibling's right to black;
        // Left rotate at parent.
        node_set_color(sibling, node_color(parent));
        node_set_black(parent);
        if (sibling->right_) node_set_black(sibling->right_);
        RotateLeft(parent);
        node = root_;
        break;
      }
    } else {
      sibling = parent->left_;

      if (node_is_red(sibling)) {
        // Case 1: Sibling is red
        node_set_red(parent);
        node_set_black(sibling);
        RotateRight(parent);
        sibling = parent->left_;
      }

      if ((!sibling->left_ || node_is_black(sibling->left_)) &&
              (!sibling->right_) ||
          node_is_black(sibling->right_)) {
        // Case 2: Sibling is black and its both child ard black
        node_set_red(sibling);
        node = parent;
        parent = node_parent(parent);
      } else {
        // Case 3,4: Sibling is black and at least one of sibling’s children is
        // red.

        if (!sibling->left_ || node_is_black(sibling->left_)) {
          // Case 3: Left is black (it means right is red, otherwise enter
          // branch "both black")
          node_set_red(sibling);
          if (sibling->right_) node_set_black(sibling->right_);

          RotateLeft(sibling);
          sibling = parent->left_;
        }

        // Case 4: Left is red, right is any.
        node_set_color(sibling, node_color(parent));
        node_set_black(parent);
        if (sibling->left_) node_set_black(sibling->left_);
        RotateRight(parent);
        node = root_;
        break;
      }
    }
  }
  // If successor is red, recoloring it to black is ok.
  // If successor is root, recolor root to black.
  if (node) node_set_black(node);
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo
