#include <sys/epoll.h>
#include <unistd.h>

#include "gtest/gtest.h"

#include "cyber/common/global_data.h"
#include "cyber/transport/common/endpoint.h"
#include "cyber/transport/common/identity.h"
#include "cyber/transport/common/syscall_wrapper.h"

namespace apollo {
namespace cyber {
namespace transport {

TEST(IdentityTest, identity_test) {
  Identity id1;
  Identity id2;
  Identity id3;
  EXPECT_NE(id1, id2);
  EXPECT_NE(id2, id3);
  EXPECT_NE(id1, id3);
  EXPECT_NE(id1.HashValue(), id3.HashValue());

  id2 = id2;
  EXPECT_NE(id2, id3);
  id2 = id3;
  EXPECT_EQ(id2, id3);

  Identity id4(id1);
  EXPECT_EQ(id1, id4);
  EXPECT_EQ(id1.ToString(), id4.ToString());
  EXPECT_EQ(id1.HashValue(), id4.HashValue());

  char* data = "endpoint";
  id4.set_data(nullptr);
  EXPECT_EQ(id1.ToString(), id4.ToString());
  id4.set_data(data);
  EXPECT_NE(id1.ToString(), id4.ToString());
  EXPECT_EQ(std::string(data), std::string(id4.data(), id4.Length()));
  AINFO << "id4's data: " << id4.data() << ", id: " << id4.ToString();

  Identity id5(false);
  EXPECT_NE(id1, id5);
}

TEST(EndpointTest, endpoint_test) {
  // empty attr
  RoleAttributes attr;
  Endpoint endpoint1(attr);
  EXPECT_EQ(common::GlobalData::Instance()->HostName(),
            endpoint1.attributes().host_name());
  EXPECT_EQ(common::GlobalData::Instance()->ProcessId(),
            endpoint1.attributes().process_id());
  EXPECT_NE(0, endpoint1.attributes().id());

  attr.set_host_name("caros");
  attr.set_process_id(1024);
  attr.set_id(123);
  Endpoint endpoint2(attr);
  EXPECT_EQ("caros", endpoint2.attributes().host_name());
  EXPECT_EQ(1024, endpoint2.attributes().process_id());
  EXPECT_EQ(123, endpoint2.attributes().id());
  EXPECT_NE(std::string("endpoint"), std::string(endpoint2.id().data()));
}

TEST(SyscallWrapperTest, syscall_wrapper_test) {
  CloseAndReset(nullptr);

  int pipe_fd[2] = {-1, -1};
  CloseAndReset(&pipe_fd[0]);
  EXPECT_FALSE(SetNonBlocking(pipe_fd[0]));

  pipe(pipe_fd);
  EXPECT_NE(pipe_fd[0], -1);
  EXPECT_NE(pipe_fd[1], -1);
  EXPECT_TRUE(SetNonBlocking(pipe_fd[0]));
  EXPECT_TRUE(SetNonBlocking(pipe_fd[1]));
  CloseAndReset(&pipe_fd[0]);
  CloseAndReset(&pipe_fd[1]);
  EXPECT_EQ(pipe_fd[0], -1);
  EXPECT_EQ(pipe_fd[1], -1);
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
