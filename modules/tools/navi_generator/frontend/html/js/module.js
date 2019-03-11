
//Vehicle data link switch control
layui.define(['form', 'jquery', 'layer'], function (exports) {
    var $ = layui.jquery,
        form = layui.form,
        layer = layui.layer;

    var obj = {
        switchingState: function (data) {
            var obj = $(data.elem);
            var name = data.elem.name;
            var state = '';
            if (data.elem.checked == true) {
                state = 'start'
            } else {
                state = 'stop'
            }
            var param = {
                "type": "requestExecuteModuleCommand",
                "module": name,
                "command": state,
            };
            // Create a Socket instance and monitor it. 
            if (!window.WebSocket) {
                window.WebSocket = window.MozWebSocket;
            }
            if (socket.readyState != 1) {
                layer.msg("Error in connection establishment .", { icon: 5 });
                obj.prop('checked', false);
                layui.form.render();
            } else {
                socket.send(JSON.stringify(param));
            }
        }
    };
    exports('module', obj);
});