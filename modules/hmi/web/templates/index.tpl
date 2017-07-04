{% extends "base.tpl" %}
{% block body %}

<style type="text/css" scoped>
  .panel_header {
    background-color: #03223e;
    overflow: auto;
    margin-top: 0;
    margin-bottom: 0;
    padding-top: 10px;
    padding-bottom: 10px;
    padding-left: 20px;
    font-size:18px;
  }

  .body_row {
    margin: 0;
    padding-right: 20px;
  }

  .body_col {
    padding-left: 20px;
    padding-right: 0;
  }

  .debug_item {
    overflow: auto;
    border: 0;
    background-color: #03294b;
    margin-bottom: 0;
    font-size: 16px;
    padding: 0;
  }

  .debug_item .item_content {
    margin: 15px 20px;
  }

  .hmi_small_btn, .hmi_small_btn:hover, .hmi_small_btn:focus {
    width: 60px;
    height: 30px;
    background: rgba(15, 62, 98, 0.34);
    border: 1px solid #30a5ff;
    border-radius: 0;
    color: #30a5ff;
  }

  .hmi_small_btn:active {
    opacity: 0.6;
  }

  .hmi_small_btn:disabled {
    opacity: 0.2;
    background: rgba(255, 255, 255, 0.20);
    border: 1px solid #ffffff;
    color: #ffffff;
  }

  .glyphicon-ok {
    color: green;
  }

  .glyphicon-info-sign {
    color: yellow;
  }

  .glyphicon-remove {
    color: red;
  }

  @keyframes rotate {
    from {transform: rotate(0deg);}
    to {transform: rotate(360deg);}
  }

  .rotating {
    animation: 1s linear 0s normal none infinite rotate;
  }
</style>

<div class="row body_row">
  <div class="col-md-4 body_col">
    {% include 'cards/quick_start_panel.tpl' %}
  </div>
  <div class="col-md-4 body_col">
    {% include 'cards/modules_panel.tpl' %}
  </div>
  <div class="col-md-4 body_col">
    {% include 'cards/hardware_panel.tpl' %}
  </div>
</div>

<script>
  // Execute the tool command at background.
  function exec_tool(tool_name) {
    // The pattern is generated at template compiling time.
    url_pattern = '{{ url_for('toolapi', tool_name='TOOL_NAME') }}';
    url = url_pattern.replace('TOOL_NAME', tool_name);
    $.get(url);
  }

  // Execute the ros request at background.
  function ros_request(cmd_name) {
    // The pattern is generated at template compiling time.
    url_pattern = '{{ url_for('rosserviceapi', cmd_name='CMD_NAME') }}';
    url = url_pattern.replace('CMD_NAME', cmd_name);
    $.get(url);
  }

  function goto_dreamview() {
    javascript:window.location.port = 8888;
  }

  // Change UI according to status.
  function on_status_change(global_status) {
    on_hardware_status_change(global_status);
    on_modules_status_change(global_status);
    on_tools_status_change(global_status);
  }

  $(document).ready(function() {
    init_modules_panel();

    // Get current status once to init UI elements.
    $.get('{{ url_for('runtimestatusapi') }}', function(json) {
      on_status_change(json);
    });

    // Setup websocket to monitor runtime status change.
    var socket = io.connect('http://' + document.domain + ':' + location.port + '/runtime_status');
    socket.on('new_status', function(json) {
      on_status_change(json);
    });
  });
</script>

<script src="{{ url_for('static', filename='lib/socket.io/1.3.6/socket.io.min.js') }}"></script>
{% endblock %}
