
const statusDiv = (
    <div>
    <h1>Rover Status</h1>
    <h2>Energy Status</h2>
    <h2>Speedometer</h2>
    <h2>Current Trip Details</h2>
    </div>
    );

ReactDOM.render(
      statusDiv,
      document.getElementById('status')
    );

const controlDiv = (
    <div>
        <h1>Mission Control</h1>
    </div>
);

ReactDOM.render(
    controlDiv,
    document.getElementById('control')
  );