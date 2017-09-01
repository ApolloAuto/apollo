<style type="text/css" scoped>
  .quick_link .btn {
    width: 100px;
    margin-top: 16px;
    margin-bottom: 10px;
    margin-right: 20px;
  }

  .right_panel .glyphicon {
    font-size: 16px;
  }
  
  .dropdown .dropdown-toggle {
    width: 180px;
  }

  .dropdown-menu {
    position: fixed;
    width: 180px;
    left: auto;
    top: auto;
    margin-right: 40px;
    background: rgba(50, 90, 125, 1);
  }

  .dropdown-menu li a {
    color: #ffffff;
  }
</style>

<div class="right_panel">
  <div class="pull-right quick_link">
    <button type="button" class="btn hmi_small_btn"
        onclick="io_request('tool_api', 'reset_all');">Reset All</button>
    <button type="button" class="btn hmi_small_btn"
        onclick="goto_dreamview()">Dreamview</button>
  </div>
  <div class="clearfix"></div>
  <h2 class="panel_header">Hardware</h2>

  <ul class="list-group">
    {% for hardware in conf_pb.hardware %}
      <li class="list-group-item debug_item
          {% if loop.index % 2 == 0 %} light {% endif %}">
        <div class="item_content" id="hardware_{{ hardware.name }}">
          {{ hardware.display_name }}
          <span class="glyphicon"></span>
          <button type="button" class="btn hmi_small_btn pull-right"
              onclick="io_request(
                  'hardware_api', 'health_check', ['{{ hardware.name }}']);"
              >Check</button>
        </div>
      </li>
    {% endfor %}
  </ul>

  <h2 class="panel_header">Configs</h2>
  <ul class="list-group">
    <li class="list-group-item debug_item light">
      <div class="item_content">Map
        <div class="dropdown pull-right">
          <button class="btn hmi_small_btn dropdown-toggle" type="button"
              data-toggle="dropdown">
              <span id="current_map">Select Map</span>
              <span class="caret"></span></button>
          <ul class="dropdown-menu">
            {% for map in conf_pb.available_maps %}
            <li><a onclick="
                io_request('tool_api', 'switch_map', ['{{ map.name }}'])">
              {{ map.name }}</a>
            </li>
            {% endfor %}
          </ul>
        </div>
      </div>
    </li>

    <li class="list-group-item debug_item">
      <div class="item_content">Vehicle
        <div class="dropdown pull-right">
          <button class="btn hmi_small_btn dropdown-toggle" type="button"
              data-toggle="dropdown">
              <span id="current_vehicle">Select Vehicle</span>
              <span class="caret"></span></button>
          <ul class="dropdown-menu">
            {% for vehicle in conf_pb.available_vehicles %}
            <li><a onclick="
                io_request('tool_api', 'switch_vehicle', ['{{ vehicle.name }}'])">
              {{ vehicle.name }}</a>
            </li>
            {% endfor %}
          </ul>
        </div>
      </div>
    </li>
  </ul>
</div>

<script>
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

  function on_config_status_change(global_status) {
    // Config status change.
    $('#current_map').text(global_status['config']['current_map']);
    $('#current_vehicle').text(global_status['config']['current_vehicle']);
  }
</script>
