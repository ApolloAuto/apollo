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

<title>Apollo Data</title>

<style>
.topic_options {
  min-width: 500px;
  font-size: 16px;
}
.page_button {
  width: 40px;
}
</style>

{% endblock %}

{% block body %}

{# Search form. #}
<form class="form-inline search_form" method="POST" action={{ url_for('search_hdl', page_idx=1) }}>
  <div class="form-group">
    <select class="form-control" name="vehicle_name">
      <option value="">Vehicle</option>
      {% for vehicle in conf.vehicles %}
        <option value="{{ vehicle }}" {% if request.vehicle_name == vehicle %} selected {% endif %}>{{ vehicle }}</option>
      {% endfor %}
    </select>
  </div>

  <div class="form-group">
    <select class="form-control" name="loop_type">
      <option value="">Loop Type</option>
      {# LoopType values are defined in task.proto. #}
      <option value="OPEN_LOOP" {% if request.loop_type == 1 %} selected {% endif %}>Open Loop</option>
      <option value="CLOSE_LOOP" {% if request.loop_type == 2 %} selected {% endif %}>Close Loop</option>
    </select>
  </div>

  <div class="form-group">
    <div class="dropdown">
      <button class="btn btn-default dropdown-toggle" type="button" data-toggle="dropdown">Topics<span class="caret"></span></button>
      <ul class="dropdown-menu topic_options">
        <li><a onclick="$('.topic_options input').prop('checked', false)">clear</a></li>
        {% for topic in conf.topics %}
          <li><div class="checkbox"><label>
            <input type="checkbox" name="topics" value="{{ topic }}"
                {% if topic in request.topics %} checked {% endif %}>
            {{ topic }}
          </label></div></li>
        {% endfor %}
      </ul>
    </div>
  </div>

  <button type="submit" class="btn btn-default">Search</button>
</form>
<br/>
{# End search form. #}

<div class="panel panel-default">
  <div class="panel-heading">Tasks</div>
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
          <th></th>
        </tr>
      </thead>
      <tbody>
        {% for task in response.tasks %}
          <tr>
            <td>{{ task.info.vehicle.name }}</td>
            <td>{{ task.start_time | timestamp_to_time }}</td>
            <td>{{ (task.end_time - task.start_time) | round(1) }} s</td>
            <td>{{ task.info.environment.map_name }}</td>
            <td>{{ task.loop_type | loop_type }}</td>
            <td>{{ task.mileage['COMPLETE_AUTO_DRIVE'] | int }} / {{ task.mileage.values() | sum | int }} m</td>
            <td><a target="_blank" href="{{ url_for('task_hdl', task_id=task.id) }}">View</a></td>
          </tr>
        {% endfor %}
      </tbody>
    </table>

    {# Pagination #}
    <script>
      function goto_page(page_idx) {
        url = '{{ url_for('search_hdl', page_idx=0) }}'.replace('/0', '/' + page_idx);
        $('.search_form').attr('action', url);
        $('.search_form').submit();
      }
    </script>
    {% for page_idx in range(1, (response.total_count - 1) // request.count + 2) %}
    <a class="btn btn-default page_button" role="button"
      {% if page_idx != (request.offset / request.count + 1) %}
        onclick="goto_page({{ page_idx }})"
      {% else %}
        disabled
      {% endif %}>{{ page_idx }}</a>
    {% endfor %}
  </div>
</div>

{% endblock %}
