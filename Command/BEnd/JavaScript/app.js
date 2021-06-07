const express = require('express');
const path = require('path');
const mqtt = require('mqtt');
const bp = require('body-parser')
const { response } = require('express');

const app = express();
const client = mqtt.connect('mqtt://54.221.168.26', {clientId:"node"});
app.use(express.urlencoded({extended: true}));
app.use(bp.json())
app.use(bp.urlencoded({ extended: true }))

// placeholder for information stored
let information = {
    speed: 1,
    battery: 1,
    ttl: 1,
    charging: 1,
    distanceTravelled: 1,
    gear: 1,
    roverCoordinates: {x: 0, y: 0},
    ballCoordinates: {},
    targetBall: 1,
    roverPath: {}

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

app.post('/api/direction', (req, res) => {

    const direction = req.body.direction;
    console.log(response.body);
    console.log(`Received new movement command: ${direction}`);

    if(client.connected === true){

        client.publish('direction', direction, () =>{
            console.log('Published direction');
        });
    }

});

app.post('/api/mode', (req, res) => {

    const mode = req.body.mode;
    console.log(`Changed mode to: ${mode}`);

    if(client.connected === true){

        client.publish('mode', mode, () =>{
            console.log('Published change in mode');
        });
    }
});

app.use(express.static(path.join(__dirname, 'views')));



client.on('connect', () =>{
    console.log('Established mqtt connection');

    client.subscribe('test', () => {
        console.log('Subscribed to test');
    });
});



client.on('message', (topic, message, packet) =>{
    console.log(`Recieved message from ${topic} - ${message} `);
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