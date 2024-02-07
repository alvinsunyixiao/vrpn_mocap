^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vrpn_mocap
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2024-02-07)
------------------
* fix readme (`#9 <https://github.com/alvinsunyixiao/vrpn_mocap/issues/9>`_)
* Add option to use VRPN timestamps rather than generating them again (`#7 <https://github.com/alvinsunyixiao/vrpn_mocap/issues/7>`_)
* fix duplicate topic name for twist (`#6 <https://github.com/alvinsunyixiao/vrpn_mocap/issues/6>`_)
  Co-authored-by: stuebema <stueben@isse.de>
  Co-authored-by: Alvin Sun <alvinsunyixiao@gmail.com>
* rename different CIs to different job names (`#8 <https://github.com/alvinsunyixiao/vrpn_mocap/issues/8>`_)
* default to use sensor data qos (`#4 <https://github.com/alvinsunyixiao/vrpn_mocap/issues/4>`_)
* Contributors: Alvin Sun, mstueben, njacquemin1993

1.0.4 (2023-04-25)
------------------
* fix readme
* use node clock (`#3 <https://github.com/alvinsunyixiao/vrpn_mocap/issues/3>`_)
* cancel ci run on previous push (`#2 <https://github.com/alvinsunyixiao/vrpn_mocap/issues/2>`_)
* add foxy ci (`#1 <https://github.com/alvinsunyixiao/vrpn_mocap/issues/1>`_)
  * add foxy ci
  * fix package name
  * revert cmake
  * downgrade cmake in CI
  * add rolling and humble CI and badages
* Contributors: Alvin Sun

1.0.3 (2022-08-30)
------------------
* fix header typo
* Contributors: Alvin Sun

1.0.2 (2022-08-30)
------------------
* fix include order to work with both rolling and foxy
* Contributors: Alvin Sun

1.0.1 (2022-08-30)
------------------
* Fix lint errors
* Contributors: Alvin Sun

1.0.0 (2022-07-04)
------------------
* Initial public release of vrpn_mocap
* Contributors: Alvin Sun
