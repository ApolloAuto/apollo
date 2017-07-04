<div>
  <style type="text/css" scoped>
    .quick_start_body {
      overflow: auto;
      background: #03223e;
      margin-bottom: 20px;
    }

    .quick_start_body .info_window {
      min-height: 60px;
      margin-left: 20px;
      margin-right: 20px;
      margin-top: 20px;
      padding: 8px;
      background: #053159;
      opacity: 0.8;
      font-size: 15px;
      letter-spacing: 0;
    }

    .quick_start_row {
      margin: 40px 20px 0px 20px;
      overflow: auto;
    }

    .quick_start_row .hmi_small_btn {
      padding: 3px 12px;
    }

    .hmi_large_btn, .hmi_large_btn:hover, .hmi_large_btn:focus {
      width: 100%;
      height: 40px;
      background: #2284d0;
      border-radius: 0;
      color: #ffffff;
    }

    .hmi_large_btn:active {
      opacity: 0.6;
    }
    
    .hmi_large_btn:disabled {
      background: #0d385f;
    }

    .quick_start_row .glyphicon {
      font-size: 24px;
      top: -6px;
      height: 24px;
    }

    .quick_start_col1 {
      padding-left: 0;
      padding-right: 13px;
      text-align: center;
    }

    .quick_start_col2 {
      padding-left: 7px;
      padding-right: 7px;
      text-align: center;
    }

    .quick_start_col3 {
      padding-left: 13px;
      padding-right: 0;
      text-align: center;
    }
  </style>

  <h3>Quick Start</h3>

  <div class="quick_start_body">
    <div class="info_window"></div>

    <h4 class="quick_start_row">
      Quick Record:
      <button type="button" id="reset_recording_btn" class="btn pull-right hmi_small_btn"
          onclick="exec_tool('reset_recording')" disabled>New</button>
    </h4>

    <div class="row quick_start_row recording">
      <div class="col-xs-4 quick_start_col1">
        <button type="button" class="btn hmi_large_btn check_btn"
            onclick="exec_tool('setup_recording')" disabled>Setup</button>
        <span class="glyphicon"></span>
      </div>
      <div class="col-xs-4 quick_start_col2">
        <button type="button" class="btn hmi_large_btn start_btn"
            onclick="exec_tool('start_recording')" disabled>Start</button>
        <span class="glyphicon"></span>
      </div>
      <div class="col-xs-4 quick_start_col3">
        <button type="button" class="btn hmi_large_btn finish_btn"
            onclick="exec_tool('stop_recording')" disabled>Stop</button>
        <span class="glyphicon"></span>
      </div>
    </div>

    <h4 class="quick_start_row">Quick Play:</h4>

    <div class="row quick_start_row playing">
      <div class="col-xs-4 quick_start_col1">
        <button type="button" class="btn hmi_large_btn check_btn"
            onclick="exec_tool('setup_playing')" disabled>Setup</button>
        <span class="glyphicon"></span>
      </div>
      <div class="col-xs-4 quick_start_col2">
        <button type="button" class="btn hmi_large_btn start_btn"
            onclick="exec_tool('start_playing'); goto_dreamview();" disabled>Start</button>
        <span class="glyphicon"></span>
      </div>
      <div class="col-xs-4 quick_start_col2">
        <button type="button" class="btn hmi_large_btn finish_btn"
            onclick="exec_tool('stop_playing')" disabled>Stop</button>
        <span class="glyphicon"></span>
      </div>
    </div>
  </div>
</div>

<script>
  last_clock_start_timestamp = undefined;
  duration = undefined;
  clock_id = undefined;

  function info_window_clock(start_timestamp, current_timestamp) {
    function show_duration() {
      var seconds = duration;
      var hours = Math.floor(seconds / 3600); seconds %= 3600;
      var minutes = Math.floor(seconds / 60); seconds %= 60;
      function two_digits(number) {
        if (number < 10) {
          return '0' + number;
        }
        return '' + number;
      }

      var duration_str = two_digits(hours) + ':' + two_digits(minutes) + ':' + two_digits(seconds);
      $('.info_window').html('Duration: ' + duration_str);
    }

    function update_clock() {
      duration += 1;
      show_duration();
    }

    if (last_clock_start_timestamp == start_timestamp) {
      return;
    } else {
      last_clock_start_timestamp = start_timestamp;
      duration = Math.floor((current_timestamp - start_timestamp) / 1000);
      show_duration();
      clock_id = setInterval(update_clock, 1000);
    }
  }

  function on_tools_status_change(global_status) {
    // Update recording buttons.
    tools_status = global_status['tools'];
    recording_status = tools_status['recording_status'];
    // Generally most buttons are disabled and without icon.
    $('.recording button').prop('disabled', true);
    $('.recording .glyphicon').attr('class', 'glyphicon');
    $('#reset_recording_btn').prop('disabled', true);

    if (recording_status == 'RECORDING_READY_TO_CHECK') {
      // Only check button is clickable.
      $('.recording .check_btn').prop('disabled', false);
    } else if (recording_status == 'RECORDING_CHECKING') {
      $('.recording .check_btn').prop('disabled', false);
      $('.recording .check_btn + .glyphicon').addClass('glyphicon-cog rotating');
    } else if (recording_status == 'RECORDING_READY_TO_START') {
      // Check button is locked, start button is clickable, finish button is disabled.
      $('.recording .check_btn + .glyphicon').addClass('glyphicon-ok');
      $('.recording .start_btn').prop('disabled', false);
    } else if (recording_status == 'RECORDING') {
      // Check and start button are locked, finish button is clickable.
      $('.recording .check_btn + .glyphicon').addClass('glyphicon-ok');
      $('.recording .start_btn + .glyphicon').addClass('glyphicon-ok');
      $('.recording .finish_btn').prop('disabled', false);
    } else if (recording_status == 'RECORDING_FINISHED') {
      // All buttons are locked, reset button is clickable.
      $('.recording .glyphicon').attr('class', 'glyphicon glyphicon-ok');
      $('#reset_recording_btn').prop('disabled', false);
    }


    // Update playing buttons.
    playing_status = tools_status['playing_status'];
    // Generally most buttons are disabled and without icon.
    $('.playing button').prop('disabled', true);
    $('.playing .glyphicon').attr('class', 'glyphicon');
    if (playing_status == 'PLAYING_READY_TO_CHECK') {
      // Only check button is clickable, no icons.
      $('.playing .check_btn').prop('disabled', false);
    } else if (playing_status == 'PLAYING_CHECKING') {
      // Only check button is clickable, with checking icon.
      $('.playing .check_btn').prop('disabled', false);
      $('.playing .check_btn + .glyphicon').addClass('glyphicon-cog rotating');
    } else if (playing_status == 'PLAYING_READY_TO_START') {
      // Check/start buttons are clickable, finish button is disabled.
      $('.playing .check_btn + .glyphicon').addClass('glyphicon-ok');
      $('.playing .start_btn').prop('disabled', false);
    } else if (playing_status == 'PLAYING') {
      // Check and start button are locked, finish button is clickable.
      $('.playing .check_btn + .glyphicon').addClass('glyphicon-ok');
      $('.playing .start_btn + .glyphicon').addClass('glyphicon-ok');
      $('.playing .finish_btn').prop('disabled', false);
    }

    // Update guide message.
    if ('message' in tools_status) {
      var msg = tools_status['message'];
      if (isNaN(msg)) {
        clearInterval(clock_id);
        $('.info_window').html(msg);
      } else {
        info_window_clock(parseInt(msg), global_status['timestamp']);
      }
    }
  }
</script>
