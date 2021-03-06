//function definitions
async function postData(url = '', data = {}) {
    console.log("Going to send: ", JSON.stringify(data));
    const response = await fetch(url, {
        method: 'POST', 
        headers: {
        'Content-Type': 'application/json'
        },
        body: JSON.stringify(data) // body dt to match header dt
    });
    let _res = response.json()
    return _res; // parses JSON response into native JavaScript objects
}

function toggleMode(){
    const modeSwitch = document.querySelector("#switch");
    const inputVal = document.querySelector("#speed-slider"); 
    const uButton = document.querySelector("#up-key"); 
    const lButton = document.querySelector("#left-key"); 
    const rButton = document.querySelector("#right-key"); 
    const dButton = document.querySelector("#up-key"); 
    const sButton = document.querySelector("#up-key"); 
    const speedControlDiv = document.querySelector('#speed-control-id');     
    if(modeSwitch.checked){
        postData("http://localhost:9000/api/mode", {mode: 1});
        console.log('Switched mode to: Autonomous');
        //alert('SWITCHING TO AUTONOMOUS: Keypad and speed slider are now disabled. Switch back to manual to resume use')
        // if Autonomous, disable manual controls
        inputVal.disabled=true;
        uButton.disabled=true;
        lButton.disabled=true;
        rButton.disabled=true;
        dButton.disabled=true;
        sButton.disabled=true;
    }else{
        postData("http://localhost:9000/api/mode", {mode: 0});
        console.log('Switched mode to: Manual');
        //alert('SWITCHING TO MANUAL: Keypad and speed slider are now enabled')
        //if Manual, enable manual controls
        inputVal.disabled=false;
        uButton.disabled=false;
        lButton.disabled=false;
        rButton.disabled=false;
        dButton.disabled=false;
        sButton.disabled=false;
    }
}

function updateDirection(keyId=''){
    const modeSwitch = document.querySelector("#switch");
    if(modeSwitch.checked==false){

        const currGear = document.querySelector("#curr-gear");
        const key = document.querySelector('#'+keyId);

        if(keyId=='up-key'){
            currGear.textContent='D'; 
            postData("http://localhost:9000/api/direction", {direction: 'w'});

        }else if(keyId=='left-key'){
            currGear.textContent='RL'; 
            postData("http://localhost:9000/api/direction", {direction: 'a'});
            
        }else if(keyId=='stop-key'){
            currGear.textContent='P'; 
            postData("http://localhost:9000/api/direction", {direction: 'x'});
            
        }else if(keyId=='right-key'){
            currGear.textContent='RR'; 
            postData("http://localhost:9000/api/direction", {direction: 'd'});
            
        }else if(keyId=='down-key'){
            currGear.textContent='R'; 
            postData("http://localhost:9000/api/direction", {direction: 's'});
            
        }else{
            console.log('Invalid key id')
        }
    }else{
        alert('ERROR: Rover on Autonomous mode, switch to manual to send direction command');
    }
    
}

function updateDirectionAuto(keyId=''){
    const modeSwitch = document.querySelector("#switch");
    if(modeSwitch.checked==true){

        const currGear = document.querySelector("#curr-gear");

        if(keyId=='w'){
            currGear.textContent='D'; 

        }else if(keyId=='a'){
            currGear.textContent='RL'; 
            
        }else if(keyId=='x'){
            currGear.textContent='P'; 
            
        }else if(keyId=='d'){
            currGear.textContent='RR'; 
            
        }else if(keyId=='s'){
            currGear.textContent='R'; 
            
        }else{
            console.log('Invalid key id')
        }
    }else{
        console.log('ERROR? Not meant to receive dir status!');
    }
    
}

function updateSpeed(){
    const modeSwitch = document.querySelector("#switch");
    if(modeSwitch.checked==false){
        const displayCurrVal = document.querySelector('#curr-speed-val');
        const inputVal = document.querySelector("#speed-slider");
        displayCurrVal.textContent = inputVal.value;
        let _speed = inputVal.value;

        postData("http://localhost:9000/api/speed", {speed: (_speed)});

        data = [{
            domain: { x: [0, 1], y: [0, 1] },
            value: _speed,
            type: "indicator",
            mode: "gauge+number",
            gauge:{
                axis: { range: [null, 18]},
                bar: { color: (_speed<10? '#3BF014' : _speed<15? '#FFFF00' : '#FF0000')},
                borderwidth: 3,
                bordercolor: "#ffebdb"
                }
        }];

        let layout = { 
            height: 275, 
            width: 550, 
            paper_bgcolor: "#25242c", 
            margin: { t: 0, b: 0 }, 
            font: { color: "#ffebdb", family: "Roboto Mono" }
        };

        // Making use of plotly's extensive API for a more lightweight way than replotting the graph
        Plotly.react('speedometer', data, layout);
        //console.log(inputVal.disabled);
    }else{       
        alert('ERROR: Rover on Autonomous mode, switch to manual to apply changes');
    }
}

function updateSpeedAuto(_speed){
    const modeSwitch = document.querySelector("#switch");
    if(modeSwitch.checked==true){
        const displayCurrVal = document.querySelector('#curr-speed-val');
        const inputVal = document.querySelector("#speed-slider");
        displayCurrVal.textContent = _speed;
        inputVal.value = _speed;

        data = [{
            domain: { x: [0, 1], y: [0, 1] },
            value: _speed,
            type: "indicator",
            mode: "gauge+number",
            gauge:{
                axis: { range: [null, 18]},
                bar: { color: (_speed<10? '#3BF014' : _speed<15? '#FFFF00' : '#FF0000')},
                borderwidth: 3,
                bordercolor: "#ffebdb"
                }
        }];

        let layout = { 
            height: 275, 
            width: 550, 
            paper_bgcolor: "#25242c", 
            margin: { t: 0, b: 0 }, 
            font: { color: "#ffebdb", family: "Roboto Mono" }
        };

        // Making use of plotly's extensive API for a more lightweight way than replotting the graph
        Plotly.react('speedometer', data, layout);
    }else{       
        console.log('ERROR: Auto update speed called in manual mode');
    }
}

function updateDistanceTraveled(distance){
    const _distance = document.querySelector("#distanceTraveled");
    _distance.textContent=distance;
}

function updateRoverCoordinates(xCoord, yCoord){
    // Plotly.update(graphDiv, data_update, layout_update, [, traceIndices])

    //update the layout and a single trace
    var dataUpdate = {'x': [[xCoord]], 'y': [[yCoord]]}
    //console.log(dataUpdate)
    //console.log(plotDiv)
    Plotly.update(plotDiv, dataUpdate,{} ,[0])

    document.querySelector('#x-coord').textContent=xCoord;
    document.querySelector('#y-coord').textContent=yCoord;
}

function addBallCoordinates(xCoord, yCoord, hexcode, colorBall){
        ballsSeen.push(colorBall);
        let newBall ={
            x:[xCoord],
            y:[yCoord],
            mode: 'markers',
            marker: {
                size: 20,
                color: `${hexcode}`
            },
            type: 'scatter',
            name: `${colorBall}`,
        };

        Plotly.addTraces(plotDiv, newBall);
}

async function fetchData(url=''){
    let res = await fetch(url);
    let data = await (res.json());
    let _data = data
    return (_data);

}

async function updateUsingFromDrive(url="http://localhost:9000/api/roverStats"){
    let res = await fetch(url);
    let roverData = await (res.json());
    updateDirectionAuto(roverData['gear']);
    updateRoverCoordinates(roverData['xCoordinate'],roverData['yCoordinate']);
    updateDistanceTraveled(roverData['distanceTravelled']);
    updateSpeedAuto(roverData['speed']);
    updateTime();


}

function ballNotification(){
    //add notification function
}

async function updateUsingFromVision(url="http://localhost:9000/api/ballStatus"){
    let res = await fetch(url);
    let ballData = await (res.json());

    if(ballsSeenCount===ballData.ballNum){
        console.log('No new ball data');
    }else{
        let colorhex;
        let _color;
        let isAColor = false;
        if(ballData.color=='r'){
            colorhex = '#FF0000';
            _color='Red';
            isAColor = true;
        }else if(ballData.color=='g'){
            colorhex = '#008080';
            _color='Green';
            isAColor = true;
        }else if(ballData.color=='b'){
            colorhex = '#0000CD';
            _color='Blue';
            isAColor = true;
        }else if(ballData.color=='v'){
            colorhex = '#C71585';
            _color='Violet';
            isAColor = true;
        }else if(ballData.color=='y'){
            colorhex = 'f5bd1f';
            _color='Yellow';
            isAColor = true;
        }
    
        if(ballsSeen.includes(_color)){
            console.log('Ball Already Plotted')
        }else if(isAColor){
            ballsSeenCount++;
            ballsSeen.push(ballData.color);
            ballHistory = ballData.archive;

            addBallCoordinates(
                ballData.ballXCoord,
                ballData.ballYCoord,
                colorhex,
                _color
            );
        }else{
            console.log('undefined color detected');
        }
    }
}

function updateTime(){
    _time += (1/600);
    document.querySelector('#time').textContent=Math.round(_time);
}

function moveToClick(){
    //manual move to point on graph feature
}

function updateChargeStatus(cnc, percentage, time){
    let lightning = document.querySelector('#cnc');
    let state = document.querySelector('#time-state');
    let percentValue = document.querySelector('#pc-val');
    let timeValue = document.querySelector('#time-till');
    let batteryLevel = document.querySelector('#battery-level');
    //Update state: Charging or not Charging
    if(cnc === 'c'){
        lightning.style.display = "inline";
        state.textContent = " until full";
    }else if( cnc = 'nc'){
        lightning.style.display = "none";
        state.textContent = " remaining";
    }

    timeValue.textContent = time>0? String(time)+' ' : "0 ";
    percentValue.textContent = percentage>0? String(percentage)+'%': "0%";
    batteryLevel.style.width = percentValue.textContent;

    if(percentage<=20){
        batteryLevel.style.backgroundColor = "red";
    }else if((percentage>20)&&(percentage<60)){
        batteryLevel.style.backgroundColor = "yellow";
    }else{
        batteryLevel.style.backgroundColor = "green";
    }

    //Update percentage and color
}

async function updateFromEnergy(url="http://localhost:9000/api/energyStatus"){
    energyfetch++;
    console.log(energyfetch);
    if(energyfetch==10){
        let res = await fetch(url);
        let energyData = await (res.json());
        console.log("RECIEVED: ", energyData);
        updateChargeStatus('c', energyData.soc, energyData.ttf);
        energyfetch=0;
    }
}

function updateFromAll(){
    updateUsingFromDrive();
    updateUsingFromVision();
    updateFromEnergy();
}


const HOST = "localhost";
const PORT = 9000;
const URL = "http://"+HOST+":"+PORT;
//const recieved = fetchData("http://localhost:9000/api/test")

let init_speed = 0;
let ballsSeenCount = 0;
let ballsSeen = [];
let ballHistory = [];
let _time = 0;
let energyfetch = 0;
// let _fromDrive = setInterval( 
//     updateFromAll ,100);

//  Speedometer 
let data = [{
domain: { x: [0, 1], y: [0, 1] },
value: init_speed,
type: "indicator",
mode: "gauge+number",
gauge:{
    axis: { range: [null, 18]},
    bar: { color: '#3BF014'},
    borderwidth: 3,
    bordercolor: "#ffebdb"
    }
}];

let layout = { 
    height: 275, 
    width: 550, 
    paper_bgcolor: "#25242c", 
    margin: { t: 0, b: 0 }, 
    font: { color: "#ffebdb", family: "Roboto Mono" }
};
Plotly.newPlot('speedometer', data, layout);

//  graphing stuff
let plotDiv = document.getElementById('map')

let rover ={
    x:[0],
    y:[0],
    mode: 'markers',
    marker: {
        size: 30,
        color: 'black'
    },
    type: 'scatter',
    name: 'Rover'
};


let data1 = [rover]//, virtualHeatmap];

let layout1 = {
    height: 700, 
    paper_bgcolor: "#ffebd9",
    plot_bgcolor: "#ffebdb",
    xaxis: {
        autorange: true
    },
    yaxis: {
        autorange: true
    }
};

Plotly.newPlot( 
    plotDiv, 
    data1,
    layout1
    );

plotDiv.on('plotly_doubleclick', function(data){
    if(document.querySelector("#switch").checked==false){    
        let xCoord2GT = data.points['x'];
        let yCoord2GT = data['points']['y'];
        console.log(xCoord2GT, yCoord2GT);
    }else{
        console.log('ERROR: Rover on Autonomous mission, await for end of mission or switch to manual'); 
    }
});
