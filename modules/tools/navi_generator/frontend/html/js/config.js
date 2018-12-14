var WebSocketServer = "ws://localhost:9888/naviGenerator";

var socket;

var files = []; // the list of files
var totalFileNum = 0;
var fileIdx = 0; // the index of the current file
var BYTES_PER_CHUNK = 1024 * 1024 * 5; // the size of each segment of a file is 5MB.
// var BYTES_PER_CHUNK = 1024 * 1;
var totalSegmentNum = 0; // the count of segments of a file
var segmentIdx = 0; // the index of the current segment
var segmentCounter = 0; // the count of segments has been read.
var start = 0;
var end = 0;
var readerArray = [];
var isOnabort = false;