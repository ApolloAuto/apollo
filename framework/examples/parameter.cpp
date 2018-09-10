
#include "cybertron/common/log.h"
#include "cybertron/cybertron.h"
#include "cybertron/parameter/parameter_client.h"
#include "cybertron/parameter/parameter_server.h"

using apollo::cybertron::Node;
using apollo::cybertron::Parameter;
using apollo::cybertron::ParameterServer;
using apollo::cybertron::ParameterClient;
int main(int argc, char *argv[]) {
  apollo::cybertron::Init(argv[0]);

  std::shared_ptr<Node> node(apollo::cybertron::CreateNode("parameter"));

  auto param_server = std::make_shared<ParameterServer>(node);
  auto param_client = std::make_shared<ParameterClient>(node, "parameter");

  param_server->SetParameter(Parameter("int", 1));
  Parameter parameter;
  param_server->GetParameter("int", &parameter);
  AINFO << "int: " << parameter.AsInt64();

  param_client->SetParameter(Parameter("string", "test"));
  param_client->GetParameter("string", &parameter);
  AINFO << "string: " << parameter.AsString();
  param_client->GetParameter("int", &parameter);
  AINFO << "int: " << parameter.AsInt64();

  apollo::cybertron::Shutdown();
  return 0;
}
