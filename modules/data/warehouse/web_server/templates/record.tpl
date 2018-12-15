{#
# Copyright 2018 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#}
{% extends "base.tpl" %}

{% block head %}

<title>Apollo Data - Record - {{ record.path }}</title>

<style>
.green {color: green;}
.red {color: red;}
.text_center {text-align: center;}
</style>
{% endblock %}

{% block body %}

<div class="panel panel-default">
  <div class="panel-heading">Information</div>
  <div class="panel-body">
    <table class="table table-striped">
      <thead>
        <tr>
          <th>Mode</th>
          <th>Map</th>
          <th>Vehicle</th>
          <th>Begin Time</th>
          <th>Duration</th>
          <th>Size</th>
          <th>Mileage (Auto/Total)</th>
        </tr>
      </thead>
      <tbody>
        <tr>
          <td>{{ record.hmi_status.current_mode }}</td>
          <td>{{ record.hmi_status.current_map }}</td>
          <td>{{ record.hmi_status.current_vehicle }}</td>
          <td>{{ record.header.begin_time | timestamp_ns_to_time }}</td>
          <td>{{ ((record.header.end_time - record.header.begin_time) / 1000000000.0) | round(1) }} s</td>
          <td>{{ record.header.size | readable_data_size }}</td>
          <td>{{ record.stat.mileages['COMPLETE_AUTO_DRIVE'] | int }} / {{ record.stat.mileages.values() | sum | int }} m</td>
        </tr>
      </tbody>
    </table>

    {# Draw map path. #}
    {% if record.stat.driving_path %}
      <script type="text/javascript" src="http://maps.google.com/maps/api/js?sensor=false&key=AIzaSyC2THXHPs0lkchGfcUOHTm-aVujoBHh2Sc"></script>
      <script type="text/javascript" src="{{ url_for('static', filename='js/gmap_util.js') }}"></script>
      <div style="width:100%; height:350px;">
        <div id="gmap_canvas" style="width: 100%; height: 100%;"></div>
        <script>
          {{ record.stat.driving_path | draw_path_on_gmap('gmap_canvas') }}
          {{ record | draw_disengagements_on_gmap }}
        </script>
      </div>

      <table class="table text_center">
        <tbody>
          <tr>
            <td><span class="glyphicon glyphicon-record green"></span> Start Point</td>
            <td><span class="glyphicon glyphicon-record red"></span> Stop Point</td>
            <td><span class="glyphicon glyphicon-map-marker red"></span> Disengagement</td>
          </tr>
        </tbody>
      </table>
    {% endif %}
  </div>
</div>

{% if sub_records %}
<div class="panel panel-default">
  <div class="panel-heading">Records</div>
  <div class="panel-body">
    <table class="table table-striped">
      <thead>
        <tr>
          <th>Begin Time</th>
          <th>Duration</th>
          <th>Size</th>
          <th>Path</th>
          <th></th>
        </tr>
      </thead>
      <tbody>
        {% for record in sub_records %}
          <tr>
            <td>{{ record.header.begin_time | timestamp_ns_to_time }}</td>
            <td>{{ ((record.header.end_time - record.header.begin_time) / 1000000000.0) | round(1) }} s</td>
            <td>{{ record.header.size | readable_data_size }}</td>
            <td>{{ record.path }}</td>
            <td><a href="{{ url_for('record_hdl', record_path=record.path[1:]) }}">View</a></td>
          </tr>
        {% endfor %}
      </tbody>
    </table>
  </div>
</div>
{% endif %}

{% if record.disengagements %}
<div class="panel panel-default">
  <div class="panel-heading">Disengagements</div>
  <div class="panel-body">
    <table class="table table-striped">
      <thead>
        <tr>
          <th>Time</th>
          <th>Description</th>
          <th>Offset</th>
        </tr>
      </thead>
      <tbody>
        {% for dis in record.disengagements %}
          <tr>
            <td>{{ dis.time | timestamp_to_time }}</td>
            <td>{{ dis.desc }}</td>
            <td>{{ (dis.time - record.header.begin_time / 1000000000.0) | round(1) }} s</td>
          </tr>
        {% endfor %}
      </tbody>
    </table>
  </div>
</div>
{% endif %}

{% if record.drive_events %}
<div class="panel panel-default">
  <div class="panel-heading">Drive Events</div>
  <div class="panel-body">
    <table class="table table-striped">
      <thead>
        <tr>
          <th>Time</th>
          <th>Type</th>
          <th>Description</th>
          <th>Offset</th>
        </tr>
      </thead>
      <tbody>
        {% for event in record.drive_events %}
          <tr>
            <td>{{ event.header.timestamp_sec | timestamp_to_time }}</td>
            <td>{% for type in event.type %} {{ type | drive_event_type_name }} {% endfor %}</td>
            <td>{{ event.event }}</td>
            <td>{{ (event.header.timestamp_sec - record.header.begin_time / 1000000000.0) | round(1) }} s</td>
          </tr>
        {% endfor %}
      </tbody>
    </table>
  </div>
</div>
{% endif %}

<div class="panel panel-default">
  <div class="panel-heading">Channels</div>
  <div class="panel-body">
    <table class="table table-striped">
      <thead>
        <tr>
          <th>Name</th>
          <th>Messages</th>
        </tr>
      </thead>
      <tbody>
        {% for channel, msg_count in record.channels.iteritems() %}
          <tr>
            <td>{{ channel }}</td>
            <td>{{ msg_count }}</td>
          </tr>
        {% endfor %}
      </tbody>
    </table>
  </div>
</div>

{% endblock %}
