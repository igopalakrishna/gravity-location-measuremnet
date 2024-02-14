const SensorData = require("../models/SensorData");
const User = require("../models/User");

const alert = async (req, res, next) => {
  const { lat, long, mobile } = req.body;
  console.log("Sensort Data received!!!");

  try {
    const sensorData = new SensorData({
      latitude: lat,
      longitude: long,
      mobile,
    });
    await sensorData.save();
    await User.findOneAndUpdate({ mobile }, { accidentOccurred: true });
    res.json({ message: "Sensor data uploaded successfully" });
  } catch (err) {
    next(err);
  }
};

module.exports = { alert };
