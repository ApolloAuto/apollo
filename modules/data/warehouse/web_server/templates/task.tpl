{#
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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

<title>Apollo Data - Task {{ task.id }}</title>

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
          <th>Vehicle</th>
          <th>Time</th>
          <th>Duration</th>
          <th>Map</th>
          <th>Type</th>
          <th>Mileage (Auto/Total)</th>
        </tr>
      </thead>
      <tbody>
        <tr>
          <td>{{ task.info.vehicle.name }}</td>
          <td>{{ task.start_time | timestamp_to_time }}</td>
          <td>{{ (task.end_time - task.start_time) | round(1) }} s</td>
          <td>{{ task.info.environment.map_name }}</td>
          <td>{{ task.loop_type | loop_type }}</td>
          <td>{{ task.mileage['COMPLETE_AUTO_DRIVE'] | int }} / {{ task.mileage.values() | sum | int }} m</td>
        </tr>
      </tbody>
    </table>

    {# Draw map path. #}
    {% if task.map_path %}
      <script type="text/javascript" src="http://maps.google.com/maps/api/js?sensor=false&key=AIzaSyC2THXHPs0lkchGfcUOHTm-aVujoBHh2Sc"></script>
      <script type="text/javascript" src="{{ url_for('static', filename='js/gmap_util.js') }}"></script>
      <div style="width:100%; height:350px;">
        <div id="gmap_canvas" style="width: 100%; height: 100%;"></div>
        <script>
          {{ task.map_path | draw_path_on_gmap('gmap_canvas') }}
          {{ task | draw_disengagements_on_gmap }}
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

<div class="panel panel-default">
  <div class="panel-heading">Rosbags</div>
  <div class="panel-body">
    <table class="table table-striped">
      <thead>
        <tr>
          <th>Start Time</th>
          <th>Duration</th>
          <th>Size</th>
          <th>Path (On NFS or HDFS)</th>
        </tr>
      </thead>
      <tbody>
        {% for bag in task.bags %}
          <tr>
            <td>{{ (bag.start_time - task.start_time) | round(1) }}</td>
            <td>{{ (bag.end_time - bag.start_time) | round(1) }} s</td>
            <td>{{ bag.size | readable_data_size }}</td>
            <td>{{ task.id | task_id_to_path }}/{{ bag.name }}</td>
          </tr>
        {% endfor %}
      </tbody>
    </table>
  </div>
</div>

<div class="panel panel-default">
  <div class="panel-heading">Disengagements</div>
  <div class="panel-body">
    <table class="table table-striped">
      <thead>
        <tr>
          <th>Time</th>
          <th>Description</th>
          <th>Bag</th>
          <th>Offset</th>
        </tr>
      </thead>
      <tbody>
        {% for dis in task.disengagements %}
          <tr>
            <td>{{ (dis.time - task.start_time) | round(1) }}</td>
            <td>{{ dis.desc }}</td>
            <td>{{ task.id | task_id_to_path }}/{{ task.bags | find_bag_by_time(dis.time) }}</td>
            <td>{{ task.bags | find_bag_offset_by_time(dis.time) | round(1) }} s</td>
          </tr>
        {% endfor %}
      </tbody>
    </table>
  </div>
</div>

<div class="panel panel-default">
  <div class="panel-heading">Topics</div>
  <div class="panel-body">
    <ul class="list-group">
      {% for topic in task.topics %}
        <li class="list-group-item">{{ topic }}</li>
      {% endfor %}
    </ul>
  </div>
</div>

{% endblock %}
