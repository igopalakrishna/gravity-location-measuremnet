const express = require("express");
const connectDB = require("./db");
const http = require("http");
const socketIo = require("socket.io");
const User = require("./models/User");
const authRoutes = require("./routes/auth");
const alertRoute = require("./routes/alert");

const app = express();
const server = http.createServer(app);
const io = socketIo(server);
const PORT = process.env.PORT || 3000;

// Connect to MongoDB
connectDB();

// Parse JSON request body
app.use(express.json());

const userChangeStream = User.watch();

userChangeStream.on("change", (change) => {
  if (
    change.operationType === "update" &&
    change.updateDescription.updatedFields.accidentOccurred === true
  ) {
    // Send an update to connected clients when an accident occurs
    console.log("Accident suspected");
    io.sockets.emit("accidentOccurred", { message: "Accident occurred!" });
  }
});

// Define routes
app.use("/auth", authRoutes);
app.use("/alert", alertRoute);

// Start the server
server.listen(PORT, () => {
  console.log(`Server started on port ${PORT}`);
});
