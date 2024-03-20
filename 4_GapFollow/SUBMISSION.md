# Lab 4: Follow the Gap

## YouTube video links
[Video 1: Levine Blocked](https://youtu.be/bpGNX8svwao)


[Video 2: Levine Obstacle](https://youtu.be/vIxzpw4fDx8)

<b>Explanation of Failure:</b>
The vehicle can effectively manuver through obstacles with the designed reactive controller, however it is unable to turn sharp enough to successfully manuever through the first turn of the levine loop, as shown in the video. The gap on the left side of car is larger than the gap to the right (which is a dead end), and thus the gap following algorithm originally selects the right turn. However, once the car progresses and decides that it instead needs to manuever to the left, it is unable to turn sharp enough. Additionally, tunning the gap depth threshold such that the right turn would not be selected  makes the car unable to manuver through the region of dense obstacles that it is spawned in.

[Video 3: Hardware Demo](https://youtu.be/--vr_0cE5nA)
