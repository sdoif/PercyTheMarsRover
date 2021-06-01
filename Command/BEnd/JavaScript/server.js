const http = require('http');
const fs = require('fs');
const _ = require('lodash');

const server = http.createServer((req, res) => {
    
    console.log('request made');
    console.log(req.url, req.method);

    //set header content type
    //res.setHeader('Content-Type', 'text/html');

    //send html file
    fs.readFile('./views/index.html', (err, data) => {

        if(err){
            console.log(err);
        }else{
            res.writeHead(200, {'Content-Type': 'text/html'});
            res.write(data);
            res.end();
        }
    });

    //res.setHeader('Content-Type', 'text/css');

    fs.readFile('./views/style.css', (err, data) => {

        if(err){
            console.log(err);
        }else{
            res.writeHead(200, {'Content-Type': 'text/css'});
            res.write(data);
            res.end();
        }
    });
});

server.listen(3000, 'localhost', () => {

    console.log('listening for reqs on port 3000');

});