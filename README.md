<p align="center">

  <img src="https://raw.githubusercontent.com/Mikuel210/Starfold/refs/heads/main/Images/Starfold.png" />

</p>

**Starfold is the flight software for [Starkit](https://github.com/Mikuel210/Starkit).**

Starkit is an electric model rocket designed for propulsive landings. Starkit is designed to test guidance, navigation and control for one day landing a rocket flying on solid motors.

Starfold is the software that will make that possible.

<p align="center">
  <img src="https://raw.githubusercontent.com/Mikuel210/Starkit/refs/heads/main/Images/39.png" />
</p>

_Starkit CAD model_

## How it works

- Starkit can detect its orientation and altitude using an IMU and a LiDAR
- Using PID algorithms, the microcontroller generates corrections to the thrust of the motors and the orientation of the TVC vanes, which control the steering of the rocket
- Having different setpoints of the PID algorithms in time means different flight profiles

## Demo video

https://youtu.be/BRE5-lADEeY

---

Made with ❤️ for Moonshot thanks to Hack Club

<div align="center">
  <a href="https://moonshot.hackclub.com" target="_blank">
    <img src="https://hc-cdn.hel1.your-objectstorage.com/s/v3/35ad2be8c916670f3e1ac63c1df04d76a4b337d1_moonshot.png" 
         alt="This project is part of Moonshot, a 4-day hackathon in Florida visiting Kennedy Space Center and Universal Studios!" 
         style="width: 100%;">
  </a>
</div>
