const mongoose = require("mongoose");

const sensorDataSchema = new mongoose.Schema(
  {
    longitude: {
      type: String,
      required: true,
    },
    latitude: {
      type: String,
      required: true,
    },
    mobile: {
      type: String,
      required: true,
    },
  },
  { timestamps: true }
);

const SensorData = mongoose.model("SensorData", sensorDataSchema);

module.exports = SensorData;
