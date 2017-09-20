import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.1


Item {

    Rectangle {
        color: "black"
        width:root.width
        height:root.height
//        Keys.enabled: true;
//        Keys.onEscapePressed: {
//            stack.pop()
//        }
        Button {
            id:backButton
            property int selected : 0
            style: ButtonStyle {
                background: Image {
                    source:"images/back.png"
                    fillMode: Image.PreserveAspectCrop
                }
            }
            x:10; y:40
            width:30; height:30
            onClicked: {
                searchBar.visible = true;
                locateBar.visible = true;
                if(stack.depth > 0)
                stack.clear();
            }
        }
        Image {
            id:searchInputImage
            x:backButton.x + 35;   y:backButton.y
            width: 30; height: 30
            source: "images/search_input.png"
            fillMode: Image.PreserveAspectCrop
        }
        /*底部蓝线*/
        Rectangle {
            width:parent.width - 100
            height: 2
            x:searchInputImage.x
            y:searchInputImage.y + 36
            color:"blue"

        }
        FocusScope {
            id: focusScope
            width:parent.width; height:30
            MouseArea {
               anchors.fill: parent
               onClicked: {
                   focusScope.focus = true;
               }
            }
            focus:true
            Keys.enabled: true;
            Keys.forwardTo: [textInput];

            TextInput {
                id:textInput
                text:"天安门"; //天安门//首都机场//马连洼梅园//燕栖湖 //五彩城
                width:parent.width
                height:parent.height
                x:searchInputImage.x + 20
                y:searchInputImage.y + 5
                color:"white"
                focus: true
                font.pixelSize:20
                Keys.enabled: true;
                Keys.onReturnPressed: {
                    mapAuto.doSearch(textInput.text);
//                    event.accepted = true;
                }
                onTextChanged: {
                    mapAuto.doSearchSug(textInput.text);
                }


            }
            Text {
                id: typeSomething
                x:textInput.x + 15; y: textInput.y
                verticalAlignment: Text.AlignVCenter
                text: "输入目的地"
                color: "gray"
            }


            Image {
                id: clear
                x:parent.width - 100; y: textInput.y
                width: 30; height: 30
                source: "images/clear.png"
                fillMode: Image.PreserveAspectCrop
                MouseArea {
                    anchors.fill: parent
                    onClicked: { textInput.text = ''; focusScope.focus = true; textInput.inputMethodHints; }
                }
            }
            Button {
                id:startSearchButton
                x:clear.x - 100; y:clear.y
                width:70; height:30
                style: ButtonStyle {
                    background: Rectangle {
                        implicitWidth: 70
                        implicitHeight: 30
                        border.width: control.pressed ?2 : 1
                        border.color: "blue"
                        color:"blue"
                        radius: 4
                        gradient: Gradient {
                            GradientStop { position: 0 ; color: control.pressed ? "#ccc" : "lightblue" }
                            GradientStop { position: 1 ; color: control.pressed ? "#aaa" : "lightblue" }
                        }
                    }
                }
                text:"开始检索"
                onClicked: {
                    mapAuto.doSearch(textInput.text);
                }
            }
            states: State {
                name: "hasText"; when: textInput.text != ''
                PropertyChanges { target: typeSomething; opacity: 0 }
                PropertyChanges { target: clear; opacity: 1 }
            }
            transitions: [
                Transition {
                    from: ""; to: "hasText"
                    NumberAnimation { exclude: typeSomething; properties: "opacity" }
                },
                Transition {
                    from: "hasText"; to: ""
                    NumberAnimation { properties: "opacity" }
                }
            ]
        }
        /////////////////////////////////////////////////
        Connections {
            target:mapAuto
                onClearSearchSugResultlist:{
                    searchSugResultModel.clear();
                }

                onSetSearchSugResultItemData:{
                    if(index == 0 || name == "") {
                        searchSugResultModel.clear();
                        return;
                    } else if(index == 1) {
                        searchSugResultModel.clear();
                    }
                    var listIndex = index + "";
                    searchSugResultModel.append({"index_numbuer":listIndex,"poiname":name,"address":address});
                }
        }
        ListModel {
            id:searchSugResultModel
        }
        ListView {
            id:searchSugReultListView
            x:textInput.x;   y:textInput.y + textInput.height
            width: textInput.width
            height: 500
            delegate: searchSugResultDelegate
            spacing:3
            orientation: ListView.Vertical
            model:searchSugResultModel
            highlight: Rectangle {
                width: 200; height: 50
                color: "#00CED1";
                radius: 5 ;
                y: searchSugReultListView.currentItem.y;
                Behavior on y { SpringAnimation { spring: 1; damping: 0.1 } }

            }
            focus:true
        }
        Component {
            id:searchSugResultDelegate
            RowLayout {
                id:listElemnetID
                ColumnLayout {
                    id:column1
                    Text {
                        id:poiName
                        font.family:"Heiti"
                        height:35
                        font.pixelSize:20
                        color:"white"
                        text:poiname
                    }
                    Text {
                        id:addr
                        font.family:"Heiti"
                        font.pixelSize:12
                        color:"gray"
                        text:address
                    }
                    Rectangle {
                        anchors.leftMargin: 35
                        y:addr.y + 40
                        width:root.width - 140
                        height: 1
                        color:"#1F2025"
                    }
                    states: State {
                        name: "Current"
                        when: listElemnetID.ListView.isCurrentItem
                        PropertyChanges { target: listElemnetID; x: 1 }
                    }
                    transitions: Transition {
                        NumberAnimation { properties: "x"; duration: 10 }
                    }
                    MouseArea {
                        anchors.fill: parent;
                        onClicked:{
                            listElemnetID.ListView.view.currentIndex = index;
                            mapAuto.doSearch(poiname);
                        }
                    }
                }
            }
        }
        /////////////////////////////////////////////////
    }
    Connections {
        target:mapAuto
            onGoResultListByWord:{
//                searchResultPage.resultRec.tet1.text = addr;
                stack.push(searchResultPage);
            }
    }
    /*Search Result View*/
    Component {
        id:searchResultPage;
        Loader {
            source:"searchResultView.qml"
        }

//        Rectangle {
//            id:resultRec
//            color: "black"
//            width:root.width
//            height:root.height
//            Text{
//                id:tet1
//                x:200
//                y:200
//                text:"addr"""
//                color:"white"
//            }
//        }
    }

}
