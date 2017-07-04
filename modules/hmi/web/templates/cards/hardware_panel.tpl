<div class="hardware_panel">
  <style type="text/css" scoped>
    .quick_link .btn {
      width: 100px;
      margin-top: 16px;
      margin-bottom: 10px;
    }

    .hardware_panel .glyphicon {
      font-size: 16px;
    }
  </style>

  <div class="pull-right quick_link">
    <button type="button" class="btn hmi_small_btn"
        onclick="exec_tool('reset_all');">Reset All</button>
    <button type="button" class="btn hmi_small_btn"
        onclick="goto_dreamview()">Dreamview</button>
  </div>
  <div class="clearfix"></div>
  <h2 class="panel_header hardware_panel">Hardware</h2>

  <ul class="list-group">
    {% for hardware in conf_pb.hardware %}
      <li class="list-group-item debug_item">
        <div class="item_content" id="hardware_{{ hardware.name }}">
          {{ hardware.display_name }}
          <span class="glyphicon"></span>
          <button type="button" class="btn hmi_small_btn pull-right"
              onclick="exec_hardware_command('{{ hardware.name }}', 'health_check');"
              >Check</button>
        </div>
      </li>
    {% endfor %}
  </ul>
</div>

<script>
  // Execute the hardware command at background.
  function exec_hardware_command(hardware_name, cmd_name) {
    // The pattern is generated at template compiling time.
    url_pattern = '{{ url_for('hardwareapi', hardware_name='HARDWARE_NAME') }}';
    url = url_pattern.replace('HARDWARE_NAME', hardware_name);
    $.post(url, {'execute_command': cmd_name});
  }

  // Change UI according to status.
  function on_hardware_status_change(global_status) {
    var hw_list = [{% for hw in conf_pb.hardware %} '{{ hw.name }}', {% endfor %}];
    hw_list.forEach(function(hw_name) {
      var status_code = undefined;
      if (hw_name in global_status['hardware']) {
        hw_status = global_status['hardware'][hw_name];
        if ('status' in hw_status) {
          status_code = hw_status['status'];
        }
      }

      $glyphicon = $("#hardware_" + hw_name + " .glyphicon");
      $glyphicon.attr("class", "glyphicon");
      if (status_code == undefined || status_code == -1) {
        return;
      }
      // See apollo::platform::hw::Status.
      if (status_code == -2) {
        // Health in checking.
        $glyphicon.addClass("glyphicon-cog rotating");
      } else if (status_code == 0) {
        // Success.
        $glyphicon.addClass("glyphicon-ok");
      } else if (status_code == 1) {
        // Not ready.
        $glyphicon.addClass("glyphicon-info-sign");
      } else {
        // Error.
        $glyphicon.addClass("glyphicon-remove");
      }
    });
  }
</script>
