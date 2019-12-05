# Motion Profile Simulator

I don't have a test bot, so here's a simulator that assumes that the robot has closed-loop velocity PID.

### Changelog

* **October 22, 2019:** fixed more `double` to `int` issues and made quicksort work with sorting 2-dimensional arrays. The following methods in `PathGenerator` are confirmed to work:
    * the constructor
    * `segV`
    * `closestPoint`
    * `curvature`
    * `getPath`
    * `Magnitude`
    * `maxVelocity`
    * `quickSort`
    * `swap`
    * `updatePos`
* **October 22, 2019:** I need help with physics, namely tankdrive physics. I have no clue how uniform circular motion works.

* **October 28, 2019:** Spend the entire day figuring out how to install FRC programs on a Windows machine. Also, GitHub is being weird with how it's storing code here so I'll upload code somehwere else.
