const mqtt = require('mqtt');

const client = mqtt.connect('mqtt://54.221.168.26', {clientId:"node"});

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
    console.log('Clearing publish');
    clearInterval(sendmessage);
}, 5000);