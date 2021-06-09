const express = require('express');
const path = require('path');
const mqtt = require('mqtt');
const bp = require('body-parser')
const { response } = require('express');

const app = express();
const client = mqtt.connect('mqtt://3.87.147.76', {clientId:"node"});
app.use(express.urlencoded({extended: true}));
app.use(bp.json())
app.use(bp.urlencoded({ extended: true }))
app.use(express.static(path.join(__dirname, 'views')));


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
    speed: 0
}

const server = app.listen(9000, (err) => {
    if(err){
        console.log('Error listening :( ', err);
        return;
    }
    console.log('Listening on port 9000');
});

app.use( (req,res,next) =>{
    console.log('Request made ');
    console.log('Host: ', req.hostname);
    console.log('Path; ', req.path);
    console.log('Method: ', req.method);
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

app.get('/api/roverStats', (req, res) => {
    console.log("Requested roverStatus");
    res.send(JSON.stringify(roverStatus));
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
});



client.on('message', (topic, message, packet) => {
    console.log(`Recieved message from ${topic} - ${message} `);
    if(topic === "drive"){
        let values = message.split("/");
        roverStatus.gear = values[0];
        roverStatus.xCoordinate = values[1];
        roverStatus.yCoordinate = values[2];
        roverStatus.distanceTravelled = values[3];
        roverStatus.speed = values[4];
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