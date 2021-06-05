const express = require('express');
const path = require('path');
const mqtt = require('mqtt');

const app = express();

app.use(express.urlencoded({extended: true}));

const server = app.listen(9000, (err) => {
    if(err){
        console.log('Error listening :( ', err);
        return;
    }
    console.log('Listening on port 3000');
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

app.use(express.static(path.join(__dirname, 'views')));