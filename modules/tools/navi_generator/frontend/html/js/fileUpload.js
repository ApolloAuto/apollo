//Modification of Section Speed Limit Value
layui.define(['jquery', 'layer', 'base64', 'resolver'], function (exports) {
    var $ = layui.jquery,
        layer = layui.layer,
        base64 = layui.base64;

    var obj = {
        generateFile: function () {
            var file = $('input[name="fileField"]').prop('files');
            if (file.length > 0) {
                $('#demoList').html("");
                for (var i = 0; i < file.length; i++) {
                    var tr = $(['<tr>'
                        , '<td>' + i + '</td>'
                        , '<td>' + file[i].name + '</td>'
                        , '<td class="transferStatus" id="fileIndex' + i + '">Waiting</td>'
                        , '</tr>'].join(''));
                    $('#demoList').append(tr);
                }
            }
        },
        modificationFile: function () {
            var file = $('input[name="modificationFiles"]').prop('files');
            if (file.length > 0) {
                $('#modificationList').html("");
                for (var i = 0; i < file.length; i++) {
                    var tr = $(['<tr>'
                        , '<td>' + i + '</td>'
                        , '<td>' + file[i].name + '</td>'
                        , '<td class="transferStatus" id="fileIndex' + i + '">Waiting</td>'
                        , '</tr>'].join(''));
                    $('#modificationList').append(tr);
                }
            }
        },
        readFileSegment: function (file, reader, start, end) {
            var blob = file.slice(start, end);
            reader.readAsBinaryString(blob);
            reader.onload = function (reader) {
                var readerData = reader.target;
                var curIndex = readerData.cur_index;
                var curSegCount = readerData.cur_seg_count;
                var curSegIdx = readerData.cur_seg_idx;
                var curName = readerData.file_name;
                var nextName = readerData.next_file_name;
                var data = base64.encode(readerData.result);
                var type = readerData.type;

                var resultMap = {
                    "type": type,
                    "curIndex": curIndex,
                    "curSegCount": curSegCount,
                    "curSegIdx": curSegIdx,
                    "curName": curName,
                    "nextName": nextName,
                    "data": data
                };

                if (!window.WebSocket) window.WebSocket = window.MozWebSocket;
                if(socket.readyState!=1){
                    layer.msg("Error in connection establishment . ", {icon: 5});
                }else{
                     socket.send(JSON.stringify(resultMap));
                }
                // console.log("send data is "+ JSON.stringify(resultMap));
                segmentCounter++;
                if (segmentCounter == curSegCount) {
                    $("#fileIndex" + fileIdx).html("Transmitted");
                    fileIdx++;
                    if (fileIdx < files.length) {
                        obj.readFileSync(files[fileIdx], type);
                    }
                    if (fileIdx == files.length) {

                        if (type == "requestCorrectRoadDeviation") {
                            $("#modificationCancel").addClass("layui-btn-disabled");
                            $("#deviationSave").removeClass("layui-btn-disabled");
                        } else {
                            $("#generateCancel").addClass("layui-btn-disabled");
                            $("#preserve").removeClass("layui-btn-disabled");
                        }
                    }
                }
            };

            reader.onerror = function (reader) {
                console.log(" error ");
                layer.alert('File read exception');
                isOnabort = true;  // TODO(*): inform the backend that the file reading is abnormal.
            };

            reader.onabort = function () {
                console.log("Transmission has been interrupted");
                isOnabort = true;
            };
        },


        readFileSync: function (file, type) {
            obj.resetSegmentVariables();
            totalSegmentNum = Math.ceil(file.size / BYTES_PER_CHUNK);
            while (start < file.size && !isOnabort) {
                var readerTmp = new FileReader();
                readerTmp["cur_index"] = fileIdx;
                readerTmp["cur_seg_count"] = totalSegmentNum;
                readerTmp["cur_seg_idx"] = segmentIdx;
                readerTmp["file_name"] = file.name;
                readerTmp["type"] = type;
                readerTmp["next_file_name"] = obj.getNextFileName();
                end = start + BYTES_PER_CHUNK;
                if (end > file.size) {
                    end = file.size;
                }
                readerTmp["start"] = start;
                readerTmp["end"] = end;
                readerArray.push(readerTmp);
                obj.readFileSegment(file, readerTmp, start, end);
                start = end;
                segmentIdx++;
            }
        },

        getNextFileName: function () {
            var nextFileName = "";
            var nextIndex = fileIdx + 1;
            if (files.length > nextIndex) {
                nextFileName = files[nextIndex].name;
            }
            return nextFileName;
        },

        upload: function (type) {
            if (!window.FileReader) {
                layer.alert('Not supported by your browser!');
                return;
            }

            if (type == "requestCorrectRoadDeviation") {
                var leftRoad = $("#modificationLeftRoad").val();
                var rightRoad = $("#modificationRightRoad").val();
            } else {
                var leftRoad = $("#LeftRoad").val();
                var rightRoad = $("#RightRoad").val();
            }


            var reg = /(^[0-9]$)|(^1[0-9]$)|(^20)$/;
            if (!reg.test(leftRoad)) {
                layer.alert('The number of left adjacent lanes must be between 0 and 20');
                return;
            }
            if (!reg.test(rightRoad)) {
                layer.alert('The number of right adjacent lanes must be between 0 and 20');
                return;
            }
            obj.resetVariables(type);

            if (type == "requestCorrectRoadDeviation") {
                files = $('input[name="modificationFiles"]').prop('files');
            } else {
                files = $('input[name="fileField"]').prop('files');
            }

            totalFileNum = files.length;
            if (totalFileNum == 0) {
                layer.alert('Please select file');
                return;
            }
            var sendHeadMap = {
                "type": type,
                "leftLaneNum": parseInt(leftRoad),
                "rightLaneNum": parseInt(rightRoad),
                "totalFileNum": totalFileNum
            };

            if (!window.WebSocket) window.WebSocket = window.MozWebSocket;
            if(socket.readyState!=1){
                layer.msg("Error in connection establishment . ", {icon: 5});
            }else{
                 socket.send(JSON.stringify(sendHeadMap));
            }
            obj.readFileSync(files[fileIdx], type);
        },

        awakeUpload: function () {
        },

        cancelUpload: function () {
            $("#generateCancel").addClass("layui-btn-disabled");

            isOnabort = true;
            for (var i = 0; i < readerArray.length; i++) {
                readerArray[i].abort();
            }

            for (var i = fileIdx; i < totalFileNum; i++) {
                $("#fileIndex" + i).html("Transmission interrupted");
            }
            // TODO(*): To inform the backend to cancel the submit
        },

        resetSegmentVariables: function () {
            readerArray = [];
            totalSegmentNum = 0;
            segmentIdx = 0;
            segmentCounter = 0;
            start = 0;
            end = 0;
        },

        resetVariables: function (type) {
            files = [];
            totalFileNum = 0;
            fileIdx = 0;
            totalSegmentNum = 0;
            segmentIdx = 0;
            segmentCounter = 0;
            start = 0;
            end = 0;
            isOnabort = false;
            if (type == "requestCorrectRoadDeviation") {
                $("#modificationCancel").removeClass("layui-btn-disabled");
            } else {
                $("#generateCancel").removeClass("layui-btn-disabled");
            }
            $(".transferStatus").html("Transmitting");
        }
    };
    exports('fileUpload', obj);
});