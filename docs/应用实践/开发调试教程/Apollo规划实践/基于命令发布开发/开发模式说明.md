## 概述

您可以通过对规划模块发布命令的方式使用规划模块，当您的场景需要进行多次规划任务才能完成一次作业要求，或者您需要在规划任务执行过程中动态改变任务状态
您可以在业务层根据您的业务需求自行编排规划任务，对规划模块发布命令。
当前 Apollo 支持以下命令：

<table>
<thead>
<tr> <th colspan="2">命令名称</th>
<th>层级</th>
<th>功能</th>
</tr>
</thead>

<tbody>
<tr>
<td colspan="2">LaneFollowCommand</td>
<td>core</td>
<td>点到点沿道路行驶</td>
</tr>

<tr>
<td colspan="2">ValetParkingCommand</td>
<td>core</td><td>泊车</td>
</tr>

<tr>
<td rowspan="6">ActionCommand</td>
<td>PULL_OVER</td>
<td rowspan="6">core</td>
<td>紧急靠边停车</td>
</tr>

<tr>
<td>STOP</td>
<td>紧急停车</td>
</tr>

<tr>
<td>START</td>
<td>继续行驶</td>
</tr>

<tr>
<td>SWITCH_TO_MANUAL</td>
<td>切换到手动模式</td>
</tr>

<tr>
<td>SWITCH_TO_AUTO</td>
<td>切换到自动模式</td>
</tr>

<tr>
<td>VIN_REQ</td>
<td>vin code验证</td>
</tr>

<tr>
<td colspan="2">ChassisCommand</td>
<td>园区</td>
<td>底盘控制命令</td>
</tr>

<tr><td colspan="2">FreeSpaceCommand</td>
<td>园区</td>
<td>非结构化道路行驶</td>
</tr>

<tr>
<td colspan="2">PathFollowCommand</td>
<td>园区</td>
<td>循迹行驶</td>
</tr>

<tr>
<td colspan="2">SpeedCommand</td>
<td>园区</td>
<td>速度限制命令</td>
</tr>
</tbody>
</table>

命令由用户业务层通过 server-client 方式发送，具体可以参考文档[《外部接口使用指南》](../附录/外部接口使用指南.md)。

您也可以基于您的场景创建新的定制命令发送给规划模块，但是 Apollo 的规划模块不会进行响应，您可以通过下一章对规划模块二次开发，创建新的 scenario，stage， task 来处理您的定制命令。

## NEXT

[Apollo规划之远程启动车辆-命令控制（9.0版）](./Apollo规划之远程启动车辆-命令控制（9.0版）.md)
