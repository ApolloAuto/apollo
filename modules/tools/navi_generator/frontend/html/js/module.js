
//Vehicle data link switch control

layui.define(['form','jquery','layer'],function(exports){
    var $ = layui.jquery,
        form = layui.form,
        layer = layui.layer;

        /*module Total selection function*/
      form.on('checkbox(allChoose)', function(data){
        var child = $(data.elem).parents('#switchForm').find('.layui-form-item input[type="checkbox"]');
        child.each(function(index, item){
          item.checked = data.elem.checked;
        });
        form.render('checkbox');
      })

      /*GPS Switching function*/
      form.on('switch(gps)', function(data){
        form.render('checkbox');
      })
       /*CAN Switching function*/
      form.on('switch(can)', function(data){
        form.render('checkbox');
      })
    var obj = {
        
    };
    exports('module', obj);
});