
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
//=======================================================================
var angle = new ROSLIB.Param({
    ros: ros,
    name: '/golf/angle'
});
function ParamangleSet() {
    obj = document.getElementsByName('ParameterElement');
    angle.set(parseFloat(obj[1].value));
}
angle.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ParameterElement");
            obj[1].value = value;
    }
});
//=======================================================================
var world_x = new ROSLIB.Param({
    ros: ros,
    name: '/golf/world_x'
});
function Paramworld_xSet() {
    obj = document.getElementsByName('ParameterElement');
    world_x.set(parseFloat(obj[2].value));
}
world_x.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ParameterElement");
            obj[2].value = value;
console.log("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
});
var world_y = new ROSLIB.Param({
    ros: ros,
    name: '/golf/world_y'
});
function Paramworld_ySet() {
    obj = document.getElementsByName('ParameterElement');
    world_y.set(parseFloat(obj[3].value));
}
world_y.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ParameterElement");
            obj[3].value = value;
    }
});
//=======================================================================
var Kx = new ROSLIB.Param({
    ros: ros,
    name: '/golf/Kx'
});
function ParamKxSet() {
    obj = document.getElementsByName('ParameterElement');
    Kx.set(parseFloat(obj[4].value));
	console.log(parseFloat(obj[4].value));
}
Kx.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ParameterElement");
            obj[4].value = value;
    }
});
var Ky = new ROSLIB.Param({
    ros: ros,
    name: '/golf/Ky'
});
function ParamKySet() {
    obj = document.getElementsByName('ParameterElement');
    Ky.set(parseFloat(obj[5].value));
}
Ky.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ParameterElement");
            obj[5].value = value;
    }
});
//=========================================================================
/*var speedlimit = new ROSLIB.Param({
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
});*/
//===============================================================
//Save
function RosSaveParam() {
    ParamHightSet();
    ParamangleSet();
    Paramworld_xSet();
    Paramworld_ySet();
    ParamKxSet();
    ParamKySet();
    //ParamSpeedLimitSet();
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
