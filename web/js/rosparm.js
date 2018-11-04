
var obj;
//=======================================================================
var hight = new ROSLIB.Param({
    ros: ros,
    name: '/golf/high'
});
function ParamHightSet() {
    obj = document.getElementsByName('ParameterElement');
    hight.set(parseFloat(obj[0].value));
}
hight.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ParameterElement");
            obj[0].value = value;
    }
});
//=========================================================================
var speedlimit = new ROSLIB.Param({
    ros: ros,
    name: '/golf/speedlimit'
});
function ParamSpeedLimitSet() {
    var box = []
    obj = document.getElementsByName('ParameterElement');
    box.push(parseFloat(obj[1].value)); 
    box.push(parseFloat(obj[2].value)); 
    speedlimit.set(box);

}
speedlimit.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ParameterElement");
            obj[1].value = value[0];
	    obj[2].value = value[1];
    }
});
//===============================================================
//Save
function RosSaveParam() {
    ParamHightSet();
    ParamSpeedLimitSet();
}

var Go = new ROSLIB.Param({
    ros: ros,
    name: '/start_go'
});

function onloadGO() {
    setTimeout(Go, 0, 0);
    setTimeout(Go, 100, 0);
    setTimeout(Go, 200, 0);
    setTimeout(Go, 300, 0);
    setTimeout(Go, 400, 0);
    setTimeout(Go, 500, 0);
}

function ParamGo(state) {
        Go.set(state);
}
