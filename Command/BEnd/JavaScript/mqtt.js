const mqtt = require('mqtt');
const crypto = require('crypto');

const client = mqtt.connect('mqtt://18.134.3.99', {clientId:"nodetest"});


let sentmessage = crypto.randomBytes(4).toString('hex');
let count = 0;

client.on('connect', () =>{
    console.log('Established mqtt connection');

    client.subscribe('test2', () => {
        console.log('Subscribed to test2');
    });
    console.log('Starting test');
    sentmessage = crypto.randomBytes(4).toString('hex');
    //sentmessage = "testing";
    console.log(sentmessage);
    client.publish('test1', sentmessage, () =>{
        console.time();
    });
});



client.on('message', (topic, message, packet) =>{

    if(count === -1){
        message = "stop";
    }
    console.log('Received Message');
    console.log(String(message));
    if(String(message) === sentmessage){
        console.log('Received correct message');
        count++;
        if(count === 1000){
            console.timeEnd();
            count = -1;
            sentmessage = "stop";
        }

    sentmessage = crypto.randomBytes(4).toString('hex');
    client.publish('test1', sentmessage);

    }

});


