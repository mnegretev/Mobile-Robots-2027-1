let ros = null
let connected = false
let cmdAckermann = null

const rosbridgeAddress = 'ws://localhost:9090'

// HTML elements
const speedSlider = document.getElementById('speed')
const steeringSlider = document.getElementById('steering')

const speedValue = document.getElementById('speed-value')
const steeringValue = document.getElementById('steering-value')

const sendButton = document.getElementById('send-button')
const stopButton = document.getElementById('stop-button')

const connectionStatus =
    document.getElementById('connection-status')


// GUI functions
function updateValues() {
    speedValue.textContent =
        parseFloat(speedSlider.value).toFixed(1) + ' m/s'

    steeringValue.textContent =
        parseFloat(steeringSlider.value).toFixed(2) + ' rad'
}


// GUI events
speedSlider.addEventListener('input', updateValues)
steeringSlider.addEventListener('input', updateValues)

sendButton.addEventListener('click', sendCommand)

stopButton.addEventListener('click', function () {
    speedSlider.value = 0
    steeringSlider.value = 0

    updateValues()
    sendCommand()
})


// ROS connection
function connectToROS() {

    ros = new ROSLIB.Ros({
        url: rosbridgeAddress
    })

    ros.on('connection', function () {

        connected = true

        connectionStatus.textContent = 'Connected'
        connectionStatus.className = 'connected'

        sendButton.disabled = false
        stopButton.disabled = false

        cmdAckermann = new ROSLIB.Topic({
            ros: ros,
            name: '/cmd_ackermann',
            messageType: 'ackermann_msgs/msg/AckermannDrive'
        })

        console.log('Connected to ROSBridge')
    })

    ros.on('error', function (error) {
        connected = false

        connectionStatus.textContent = 'Connection error'
        connectionStatus.className = 'disconnected'

        sendButton.disabled = true
        stopButton.disabled = true

        console.error('ROSBridge error:', error)
    })

    ros.on('close', function () {
        connected = false
        cmdAckermann = null

        connectionStatus.textContent = 'Disconnected'
        connectionStatus.className = 'disconnected'

        sendButton.disabled = true
        stopButton.disabled = true

        console.log('ROSBridge connection closed')
    })
}


// Publish Ackermann command
function sendCommand() {

    if (!connected || cmdAckermann === null) {
        console.log('ROS is not connected')
        return
    }

    const speed = parseFloat(speedSlider.value)
    const steering = parseFloat(steeringSlider.value)

    const message = new ROSLIB.Message({
        steering_angle: steering,
        steering_angle_velocity: 0.0,
        speed: speed,
        acceleration: 0.0,
        jerk: 0.0
    })

    cmdAckermann.publish(message)

    console.log(
        `Command sent: speed=${speed} m/s, steering=${steering} rad`
    )
}


// Start connection
connectToROS()