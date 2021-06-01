const express = require('express');
const path = require('path');

const app = express();

const server = app.listen(3000, () => {

    console.log('Listening on port 3000');

});

app.get('/', (req, res) => {

    res.sendFile('./views/index.html', { root: __dirname });

});

app.use(express.static(path.join(__dirname, 'views')));