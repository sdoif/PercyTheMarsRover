const express = require('express');
const path = require('path');
const mqtt = require('mqtt');
const bp = require('body-parser')
const { response, query } = require('express');
const { Console } = require('console');
const sqlite3 = require('sqlite3')

const app = express();
const client = mqtt.connect('mqtt://18.134.3.99', {clientId:"node"});
app.use(express.urlencoded({extended: true}));
app.use(bp.json())
app.use(bp.urlencoded({ extended: true }))
app.use(express.static(path.join(__dirname, 'views')));

let db = new sqlite3.Database(path.join(__dirname, '..', '..', 'database', 'energyDB.sl3'));

let timeCounter = 1;

// placeholder for information stored
let information = {
    battery: 0,
    ttl: 0,
    charging: 0,
    ballCoordinates: {},
    targetBall: 0,
    roverPath: {}
}

let roverStatus = {

    gear: "x",
    xCoordinate: 0,
    yCoordinate: 0,
    distanceTravelled: 0,
    speed: 0,
    ballsSeen: 0
}

let ballStatus={
    color: '',
    distance: 0,
    ballX: 0,
    ballY: 0,
    count: 0
}

let roverBallOrientation={
    theta: 0,
    xCoord: 0,
    yCoord: 0
}

let allBallsSeen=[];

let roverStatusInit = roverStatus;

const server = app.listen(9000, (err) => {
    if(err){
        console.log('Error listening :( ', err);
        return;
    }
    console.log('Listening on port 9000');
});

app.use( (req,res,next) =>{
    // console.log('Request made ');
    // console.log('Host: ', req.hostname);
    // console.log('Path; ', req.path);
    // console.log('Method: ', req.method);
    next();
});

app.get('/', (req, res) => {

    res.sendFile('./views/index.html', { root: __dirname });
    console.log(req.url);
});

app.get('/api/test', (req, res) => {
    console.log("Sending", JSON.stringify('Testing'))
    res.send(JSON.stringify('Testing'));
    console.log('entered test');

});

// schema
// charging(
//     time INTEGER PRIMARY KEY,
//     soc INTEGER,
//     ttf INTEGER
//     );
let sql = `SELECT soc, ttf FROM charging
           WHERE time = ?;`;


app.get('/api/energyStatus', (req, res) => {
    if(timeCounter<3063){
        db.all(sql, [timeCounter], (err,row) => {
            if(err){
                console.log(err.message);
            }
            let queryRes = row[0];
            console.log("ENERGY FETCHED as: ", queryRes);
            //console.log("Requested energyStatus");
            res.send(JSON.stringify(queryRes));
        });
        timeCounter++;
    }else{
        db.close();
    }
});

app.get('/api/roverStats', (req, res) => {
    //console.log("Requested roverStatus");
    res.send(JSON.stringify(roverStatus));
});

app.get('/api/ballStatus', (req, res) => {
    let newBallStatus = {
        ballXCoord : ballStatus.ballX,
        ballYCoord : ballStatus.ballY,
        color : ballStatus.color,
        ballNum : ballStatus.count,
        archive: allBallsSeen
    }
    //cconsole.log("Requested ballStatus");
    if(newBallStatus.color==='r'||'g'||'y'||'b'||'v'){
        res.send(JSON.stringify(newBallStatus));
    }
});

app.post('/api/direction', (req, res) => {

    const direction = req.body.direction;
    console.log(response.body);
    console.log(`Received new movement command: ${direction}`);
    res.json("Command Received");

    if(client.connected === true){

        client.publish('direction', direction, () =>{
            console.log('Published direction');
        });
    }

});

app.post('/api/speed', (req, res) => {

    const speed = req.body.speed.toString();
    console.log(`Received new speed setting: ${speed}`);
    res.json("Speed Changed");
    console.log(typeof(speed));
    let decimal = "0";
    let units = "00";

    if(speed.length === 4){
        decimal = speed.charAt(3);
        units = speed.substring(0,2);
    }else if(speed.length === 3){
        decimal = speed.charAt(2);
        units = speed.charAt(0);
        units = 0 + units;
    }else if(speed.length === 2){
        units = speed;
    }else{
        units = speed.charAt(0);
        units = 0 + units
    }

    let readspeed = units + decimal;

    console.log(readspeed);

    if(client.connected === true){

        client.publish('speed', readspeed, () =>{
            console.log('Published new speed');
        });
    }

});

app.post('/api/mode', (req, res) => {

    const _mode = req.body.mode;
    const mode = String(_mode);
    console.log(`Changed mode to: ${mode}`);
    res.json("Mode Changed");

    if(client.connected === true){

        client.publish('mode', mode, () =>{
            console.log('Published change in mode');
        });
    }
});


client.on('connect', () =>{
    console.log('Established mqtt connection');

    client.subscribe('test', () => {
        console.log('Subscribed to test');
    });

    client.subscribe('drive', () => {
        console.log('Subscribed to drive');
    });

    client.subscribe('vision', () => {
        console.log('Subscribed to vision');
    });

    client.subscribe('ball', () => {
        console.log('Subscribed to ball');
    });
});



client.on('message', (topic, message, packet) => {
    strMessage = message.toString();
    //console.log(strMessage)
    let values = strMessage.split("/");
    if(topic === "drive"){
        let values = strMessage.split("/");
        roverStatus.gear = (values[1]);
        roverStatus.xCoordinate = (values[2])===undefined? roverStatus.xCoordinate : Number(values[2]) ;
        roverStatus.yCoordinate = (values[3])===undefined? roverStatus.yCoordinate : Number(values[3]) ;
        roverStatus.distanceTravelled = (values[4])===undefined? roverStatus.distanceTravelled : Number(values[4]) ;
        roverStatus.speed = (values[5])===undefined? roverStatus.speed : Number(values[5]) ;
        //console.log("Rover status changed to: ", roverStatus);
    }else if(topic === "vision"){
        console.log(`Recieved message from ${topic} - ${strMessage}`);
        //parse values from vision to make them ready for getting
        ballStatus.color = (values[1]);
        ballStatus.distance = Number(values[2]);
        //Calculate ball coordinates
        ballStatus.ballX = roverStatus.xCoordinate  + (ballStatus.distance * Math.cos(roverBallOrientation.theta)) ;
        ballStatus.ballY = roverStatus.yCoordinate + (ballStatus.distance * Math.sin(roverBallOrientation.theta)) ;
        //let frontend know it is time to make a request
        ballStatus.count++;
        allBallsSeen.push({color: ballStatus.color, xCoordinate: ballStatus.ballX, yCoordinate: ballStatus.ballY})
        console.log("New ball alert! : ", ballStatus);
    }else if(topic === "ball"){
        console.log(`Recieved message from ${topic} - ${strMessage}`);
        roverBallOrientation.theta = Number(values[1]);
        roverBallOrientation.xCoord = Number(values[2]);
        roverBallOrientation.yCoord = Number(values[3]);
    }
    
});




const sendmessage = setInterval(publishMessage, 2000);

function publishMessage() {

    if(client.connected === true){

        client.publish('test', 'hi', () =>{
            console.log('Published to test');
        });
    };
};

setTimeout( () => {
    console.log('Clearing sendmessage');
    clearInterval(sendmessage);
}, 5000);