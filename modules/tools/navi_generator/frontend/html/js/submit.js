
//Vehicle data acquisition
layui.define(['form','jquery','layer'],function(exports){
    var $ = layui.jquery,
        form = layui.form,
        layer = layui.layer;

        //Data parameter control
        form.on('radio(first-input)', function(data){
          $(this).parents(".layui-inline").find("input[name='inputMin']").val("").addClass("layui-disabled").attr("disabled",true);
          $(this).parents(".layui-inline").find("input[name='inputKm']").val("").addClass("layui-disabled").attr("disabled",true);
        });
        form.on('radio(minSpeed)', function(data){
          $(this).parents(".minSpeed").find("input[name='inputMinimum']").val("").addClass("layui-disabled").attr("disabled",true);
        });
        form.on('radio(maxSpeed)', function(data){
          $(this).parents(".maxSpeed").find("input[name='inputMaximum']").val("").addClass("layui-disabled").attr("disabled",true);
        });
        /*Single input box event handling*/
        form.on('radio(collect)', function(data){ 
          $(this).parents(".layui-inline").find(".layui-input").val("").addClass("layui-disabled").attr("disabled",true);
          $(this).siblings("input").removeClass("layui-disabled").removeAttr("disabled");
          });
    
    //Release to objects that need to receive files.         
    var obj = {
        Recording:function (data,type){
          var sexVal,minSpeedVal,maxSpeedVal;
          //Background receiving data format judgement            
          if(data.field.sex==""&&data.field.inputMin==""&&data.field.inputKm==""){
            layer.alert("The value of collection type can not be empty");
          }else if(data.field.inputMin!=""){
            sexVal=data.field.inputMin;
          }else if(data.field.inputKm!=""){
            sexVal=data.field.inputKm;
          }else{
            sexVal=data.field.sex;
          }
          if(data.field.minSpeed==""&&data.field.inputMinimum==""){
            layer.alert("The value of Minimum speed can not be empty");
          }else if(data.field.inputMinimum!==""){
            minSpeedVal=data.field.inputMinimum;
          }else{
            minSpeedVal=data.field.minSpeed;
          }
          if(data.field.maxSpeed==""&&data.field.inputMaximum==""){ 
            layer.alert("The value of Maximum speed can not be empty");
          }else if(data.field.inputMaximum!==""){
            maxSpeedVal=data.field.inputMaximum; 
          }else{
            maxSpeedVal=data.field.maxSpeed;
          }
         $(':checked').each(function() {
              var name = $(this).attr('data-type');
              if(name=="min"){
                sexVal=sexVal+"min";
              }else if(name=="km"){
                sexVal=sexVal+"km";
              }
            });
          var param = {
              "collectionType": sexVal,
              "lowSpeedLimit": Number(minSpeedVal),
              "highSpeedLimit": Number(maxSpeedVal),
              "type":type
          };
         // Create a Socket instance and monitor it. 
          if (!window.WebSocket) {
              window.WebSocket = window.MozWebSocket;
          }
          if(socket.readyState!=1){
              layer.msg("Error in connection establishment .", {icon: 5});
          }else{
              socket.send(JSON.stringify(param));
          }
        }
    };
    exports('submit', obj);
});