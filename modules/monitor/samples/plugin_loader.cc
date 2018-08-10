#include <iostream>
#include <stdio.h>
#include <dlfcn.h>

#include "modules/monitor/sysmon/base/plugin_interface.h"

int main(int argc, const char *argv[])
{
  void *dl_hdl = dlopen("libdummy_plugin.so", RTLD_LAZY | RTLD_LOCAL);
  if (!dl_hdl) {
    std::cout << "failed to load dll " << dlerror() << std::endl;
    return -1;
  }

  apollo::monitor::sysmon::MetricQuerentPluginEntry *tx =
      (apollo::monitor::sysmon::MetricQuerentPluginEntry*)dlsym(dl_hdl, "sysmon_metric_querent_list");
  printf("got: %p\n", tx);
  for (int i; ; ++i) {
    if (tx[i].name) {
      printf("%s: %p\n", tx[i].name, &(tx[i].inst_creator));
    } else {
      break;
    }
  }

  dlclose(dl_hdl);
  return 0;
}
