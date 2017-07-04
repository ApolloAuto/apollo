<div>
  <style type="text/css" scoped>
    .module_switch {
      width: 40px;
      height: 20px;
      border-radius: 62px;
    }

    .module_switch .module_switch_btn {
      width: 16px;
      height: 16px;
      margin: 2px;
      border-radius: 100%;
    }

    .module_switch_open {
      background: #0e3d62;
    }

    .module_switch_open .module_switch_btn {
      float: right;
      background: #30a5ff;
    }

    .module_switch_close {
      background: #2a2e30;
    }

    .module_switch_close .module_switch_btn {
      float: left;
      background: #6e6e6e;
    }
</style>

  <h3>Debug</h3>
  <h2 class="panel_header">Modules</h2>

  <ul class="list-group">
    {% for module in conf_pb.modules %}
    <li class="list-group-item debug_item">
      <div class="item_content" id="module_{{ module.name }}">
        {{ module.display_name }}
        <div class="pull-right module_switch module_switch_close"
            ><div class="module_switch_btn"></div></div>
      </div>
    </li>
    {% endfor %}
  </ul>

</div>

<script>
  // Execute the module command at background.
  function exec_module_command(module_name, cmd_name) {
    // The pattern is generated at template compiling time.
    url_pattern = '{{ url_for('moduleapi', module_name='MODULE_NAME') }}';
    url = url_pattern.replace('MODULE_NAME', module_name);
    $.post(url, {'execute_command': cmd_name});
  }

  function init_modules_panel() {
    var modules = [{% for module in conf_pb.modules %} '{{ module.name }}', {% endfor %}];
    modules.forEach(function(module_name) {
      // Init module switches.
      $btn = $("#module_" + module_name + " .module_switch");
      $btn.click(function(e) {
        if ($(this).hasClass("module_switch_close")) {
          exec_module_command(module_name, 'start');
        } else {
          exec_module_command(module_name, 'stop');
        }
        $(this).toggleClass("module_switch_close module_switch_open");
      });
    });
  }

  // Change UI according to status.
  function on_modules_status_change(global_status) {
    var modules = [{% for module in conf_pb.modules %} '{{ module.name }}', {% endfor %}];
    modules.forEach(function(module_name) {
      // Get module status.
      status_code = 'UNINITIALIZED';
      if (module_name in global_status['modules']) {
        module_status = global_status['modules'][module_name];
        if ('status' in module_status) {
          status_code = module_status['status'];
        }
      }

      $btn = $("#module_" + module_name + " .module_switch");
      if (status_code != 'STARTED') {
        $btn.removeClass("module_switch_open").addClass("module_switch_close");
      } else {
        $btn.removeClass("module_switch_close").addClass("module_switch_open");
      }
    });
  }
</script>
