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

<div class="panel panel-default">
  <div class="panel-heading">{% if is_tasks %} Tasks {% else %} Records {% endif %}</div>
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
          <th>Issues</th>
          <th>Mileage (Auto/Total)</th>
          <th></th>
        </tr>
      </thead>
      <tbody>
        {% for record in records %}
          <tr>
            <td>{{ record.hmi_status.current_mode }}</td>
            <td>{{ record.hmi_status.current_map }}</td>
            <td>{{ record.hmi_status.current_vehicle }}</td>
            <td>{{ record.header.begin_time | timestamp_ns_to_time }}</td>
            <td>{{ ((record.header.end_time - record.header.begin_time) / 1000000000.0) | round(1) }} s</td>
            <td>{{ record.header.size | readable_data_size }}</td>
            <td>{{ (record.disengagements | length) + (record.drive_events | length) }}</td>
            <td>{{ record.stat.mileages['COMPLETE_AUTO_DRIVE'] | int }} / {{ record.stat.mileages.values() | sum | int }} m</td>
            <td><a target="_blank"
              {% if is_tasks %}
                href="{{ url_for('task_hdl', task_path=record.path[1:]) }}"
              {% else %}
                href="{{ url_for('record_hdl', record_path=record.path[1:]) }}"
              {% endif %}
            >View</a></td>
          </tr>
        {% endfor %}
      </tbody>
    </table>

    {# Pagination #}
    {% for page_idx in range(1, page_count + 1) %}
    <a class="btn btn-default page_button" role="button"
      {% if page_idx != current_page %}
        {% if is_tasks %}
          href="{{ url_for('tasks_hdl', page_idx=page_idx) }}"
        {% else %}
          href="{{ url_for('records_hdl', page_idx=page_idx) }}"
        {% endif %}
      {% else %}
        disabled
      {% endif %}>{{ page_idx }}</a>
    {% endfor %}
  </div>
</div>

{% endblock %}
