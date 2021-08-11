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

#include "cyber/service_discovery/container/graph.h"

#include <string>
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

TEST(GraphTest, vertice) {
  Vertice a;
  EXPECT_TRUE(a.IsDummy());

  Vertice b(a);
  EXPECT_TRUE(b.IsDummy());
  EXPECT_EQ(b, a);

  Vertice c = a;
  EXPECT_TRUE(c.IsDummy());
  EXPECT_EQ(c.GetKey(), a.GetKey());
  a = a;

  Vertice d("d");
  EXPECT_FALSE(d.IsDummy());
  EXPECT_NE(d, a);
  EXPECT_NE(d.GetKey(), a.GetKey());
  EXPECT_EQ(d.value(), "d");
}

TEST(GraphTest, edge) {
  Edge a;
  EXPECT_FALSE(a.IsValid());

  Edge b(a);
  EXPECT_FALSE(b.IsValid());
  EXPECT_EQ(b, a);
  EXPECT_EQ(b.GetKey(), a.GetKey());

  Edge c = a;
  EXPECT_FALSE(c.IsValid());
  EXPECT_EQ(c, a);
  EXPECT_EQ(c.GetKey(), a.GetKey());
  a = a;

  Edge d;
  d = c;
  EXPECT_FALSE(d.IsValid());
  EXPECT_EQ(d, c);
  EXPECT_EQ(d.GetKey(), c.GetKey());

  const std::string value = "value";
  a.set_value(value);
  EXPECT_EQ(a.value(), value);
  EXPECT_FALSE(a == b);
  EXPECT_FALSE(a.IsValid());
  b.set_value(value);
  EXPECT_EQ(a, b);

  Vertice src("src");
  Vertice dst("dst");
  a.set_src(src);
  b.set_dst(dst);
  EXPECT_EQ(a.src(), src);
  EXPECT_EQ(b.dst(), dst);
  EXPECT_FALSE(a == b);
  EXPECT_TRUE(a.IsValid());
  EXPECT_TRUE(b.IsValid());

  a.set_dst(dst);
  b.set_src(src);
  EXPECT_EQ(a, b);
  EXPECT_EQ(a.GetKey(), "value_dst");

  Edge e(src, dst, value);
  EXPECT_EQ(e, a);
}

TEST(GraphTest, graph) {
  Graph g;
  EXPECT_EQ(g.GetNumOfEdge(), 0);

  Vertice dummy;
  Vertice a("a");
  Vertice b("b");

  Edge ab;
  g.Insert(ab);
  EXPECT_EQ(g.GetNumOfEdge(), 0);
  ab.set_src(a);
  ab.set_dst(b);
  ab.set_value("ab");
  g.Insert(ab);
  EXPECT_EQ(g.GetNumOfEdge(), 1);
  // repeated insert
  g.Insert(ab);
  EXPECT_EQ(g.GetNumOfEdge(), 1);

  Vertice c("c");
  Vertice d("d");
  Edge cd(c, d, "cd");
  g.Insert(cd);
  EXPECT_EQ(g.GetNumOfEdge(), 2);

  Vertice e("e");
  Edge ce(c, e, "ce");
  g.Insert(ce);
  EXPECT_EQ(g.GetNumOfEdge(), 3);

  Edge ac;
  ac.set_src(a);
  ac.set_dst(dummy);
  ac.set_value("ac");
  g.Insert(ac);
  EXPECT_EQ(g.GetNumOfEdge(), 3);

  ac.set_src(dummy);
  ac.set_dst(c);
  g.Insert(ac);
  EXPECT_EQ(g.GetNumOfEdge(), 4);

  Edge be;
  be.set_src(dummy);
  be.set_dst(e);
  be.set_value("be");
  g.Insert(be);
  EXPECT_EQ(g.GetNumOfEdge(), 4);

  be.set_dst(dummy);
  be.set_src(b);
  g.Insert(be);
  EXPECT_EQ(g.GetNumOfEdge(), 5);

  EXPECT_EQ(g.GetDirectionOf(a, b), UPSTREAM);
  EXPECT_EQ(g.GetDirectionOf(b, a), DOWNSTREAM);
  EXPECT_EQ(g.GetDirectionOf(a, c), UPSTREAM);
  EXPECT_EQ(g.GetDirectionOf(c, a), DOWNSTREAM);
  EXPECT_EQ(g.GetDirectionOf(b, e), UPSTREAM);
  EXPECT_EQ(g.GetDirectionOf(e, b), DOWNSTREAM);
  EXPECT_EQ(g.GetDirectionOf(c, d), UPSTREAM);
  EXPECT_EQ(g.GetDirectionOf(d, c), DOWNSTREAM);
  EXPECT_EQ(g.GetDirectionOf(c, e), UPSTREAM);
  EXPECT_EQ(g.GetDirectionOf(e, c), DOWNSTREAM);
  EXPECT_EQ(g.GetDirectionOf(a, e), UPSTREAM);
  EXPECT_EQ(g.GetDirectionOf(e, a), DOWNSTREAM);
  EXPECT_EQ(g.GetDirectionOf(a, d), UPSTREAM);
  EXPECT_EQ(g.GetDirectionOf(d, a), DOWNSTREAM);

  EXPECT_EQ(g.GetDirectionOf(b, d), UNREACHABLE);
  EXPECT_EQ(g.GetDirectionOf(d, b), UNREACHABLE);
  EXPECT_EQ(g.GetDirectionOf(d, e), UNREACHABLE);
  EXPECT_EQ(g.GetDirectionOf(e, d), UNREACHABLE);

  EXPECT_EQ(g.GetDirectionOf(dummy, dummy), UNREACHABLE);
  EXPECT_EQ(g.GetDirectionOf(a, dummy), UNREACHABLE);
  EXPECT_EQ(g.GetDirectionOf(dummy, a), UNREACHABLE);

  Vertice f("f");
  EXPECT_EQ(g.GetDirectionOf(a, f), UNREACHABLE);
  EXPECT_EQ(g.GetDirectionOf(f, a), UNREACHABLE);

  g.Delete(be);
  EXPECT_EQ(g.GetDirectionOf(b, e), UNREACHABLE);
  EXPECT_EQ(g.GetDirectionOf(e, b), UNREACHABLE);
  EXPECT_EQ(g.GetNumOfEdge(), 4);
  g.Delete(be);

  g.Delete(ac);
  EXPECT_EQ(g.GetNumOfEdge(), 3);
  EXPECT_EQ(g.GetDirectionOf(a, c), UNREACHABLE);
  EXPECT_EQ(g.GetDirectionOf(d, a), UNREACHABLE);

  g.Delete(cd);
  EXPECT_EQ(g.GetNumOfEdge(), 2);
  EXPECT_EQ(g.GetDirectionOf(c, d), UNREACHABLE);
  EXPECT_EQ(g.GetDirectionOf(d, c), UNREACHABLE);

  g.Delete(ab);
  EXPECT_EQ(g.GetNumOfEdge(), 1);

  g.Delete(ce);
  EXPECT_EQ(g.GetNumOfEdge(), 0);

  Vertice q("q");
  Edge qa;
  g.Delete(qa);
  qa.set_src(q);
  qa.set_dst(a);
  qa.set_value("qa");
  g.Delete(qa);
}

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
