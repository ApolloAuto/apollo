
//Navigation Line Generator JS control entry
layui.config({
    base : "js/"
}).use(['form','jquery','layer','map','module','submit',"modifySpeed","fileUpload","preserve","route",'resolver'],function(exports){
  var $ = layui.jquery,
      form = layui.form,
      layer = layui.layer,
      map = layui.map,
      module = layui.module,
      submit = layui.submit,
      modifySpeed = layui.modifySpeed,
      preserve = layui.preserve,
      route = layui.route,
      resolver = layui.resolver,
      fileUpload = layui.fileUpload;
      map.map();

    socket = new WebSocket(WebSocketServer);

    socket.onopen = function (event) {
        console.log("websocket connected success");
    };

    socket.onmessage = function (event) {
        var data = $.parseJSON(event.data);
        resolver.parseData(data,fileUpload);
    };
    socket.onerror = function(event){
        console.log("socket Error ");
    };

    $("#left-side li").click(function (){
      $(this).siblings('li').removeClass('active');
      $(this).addClass('active');
    });

    $('.choose').click(function () {
      $("#collect").animate({height:'22%'});
      $("#editor").animate({height:'0'});
    });
    $('.pay').click(function () {
      $("#collect").animate({height:'0'});
      $("#editor").animate({height:'22%'});
    });

    $(".closeFileWin").click(function(){
      $("#modification").hide();
      $("#generate").hide();
      $("#speedModification").hide();
    });
    //Data entry submission
    form.on('submit(formStart)', function(data){
      submit.Recording(data,"requestStartCollection");
      return false;
    });
    form.on('submit(formChange)', function(data){
      submit.Recording(data,"requestUpdateCollectionCondition");
      return false;
    });
    form.on('submit(formEnd)', function(){
      if (!window.WebSocket) {
          window.WebSocket = window.MozWebSocket;
      }
      if(socket.readyState!=1){
          layer.msg("Error in connection establishment .", {icon: 5});
      }else{
          socket.send(JSON.stringify("requestFinishCollection"));
      }
    });


    //generate file show
    $(document).on('click ','#mapFile',function(){
      dataClear();
      $("#modification").hide();
      $("#speedModification").hide();
      $("#generate").show();
    });

   // Generate File selection
   $(document).on('change','#files',function(evt){
        fileUpload.generateFile(evt);
   });

    //generate File upload
   $(document).on('click ','#generateSubmit',function(){
        map.remove();
        fileUpload.upload("requestProcessBagFiles");
   });

    //generateCancel File upload cancel
    $(document).on('click ','#generateCancel',function(){
        fileUpload.cancelUpload();
    });

     //generateCancel File preserve
    $(document).on('click ','#preserve',function(){
        preserve.preserve("requestSaveBagFiles");
    });

    //clearOverlays
    $(document).on('click ','#closeMarker',function(){
        map.remove();
   });



    //modification file show
    $(document).on('click ','#deviation',function(){
      dataClear();
      $("#generate").hide();
      $("#speedModification").hide();
      $("#modification").show();
    });

    // Generate File selection
     $(document).on('change','#modificationFiles',function(evt){
          fileUpload.modificationFile(evt);
     });

    //Modify deviation upload
    $(document).on('click ','#modificationSubmit',function(){
        map.remove();
        fileUpload.upload("requestCorrectRoadDeviation");
    });

    //modificationCancel File upload cancel
    $(document).on('click ','#modificationCancel',function(){
        fileUpload.cancelUpload();
    });

    //deviation File save
    $(document).on('click ','#deviationSave',function(){
        preserve.preserve("requestSaveRoadCorrection");
    });

    //speed show
    $(document).on('click ','#speed',function(){
      dataClear();
      $("#generate").hide();
      $("#modification").hide();
      $("#speedModification").show();
    });

    //Modify speed limit
    $(document).on('click','#speedSubmit',function(){
        modifySpeed.pointSelect(map.getMarkerVal());
    });

    //speedSave
    $(document).on('click','#speedSave',function(){
        preserve.preserve("requestSaveSpeedLimitCorrection");
    });


     //Route planning submission
    $(document).on('click','#route',function(){
        route.planning(map.getMarkerVal(),map.getBMap(),"requestRoute");
    });

    $(document).on('click','#navigate',function(){
        route.planning(map.getMarkerVal(),map.getBMap(),"requestNavigation");
    });

    function dataClear(){
      $('#demoList').html("");
      $("#LeftRoad").val("0");
      $("#RightRoad").val("0");
      $('#modificationList').html("");
      $("#modificationLeftRoad").val("0");
      $("#modificationRightRoad").val("0");
      $("#modify-minSpeed").val("");
      $("#modify-maxSpeed").val("");
    }

})