
"use strict";

let MissionWaypoint = require('./MissionWaypoint.js');
let MissionHotpointTask = require('./MissionHotpointTask.js');
let WaypointList = require('./WaypointList.js');
let PayloadData = require('./PayloadData.js');
let MobileData = require('./MobileData.js');
let FCTimeInUTC = require('./FCTimeInUTC.js');
let VOPosition = require('./VOPosition.js');
let GPSUTC = require('./GPSUTC.js');
let MissionWaypointTask = require('./MissionWaypointTask.js');
let Gimbal = require('./Gimbal.js');
let MissionWaypointAction = require('./MissionWaypointAction.js');
let FlightAnomaly = require('./FlightAnomaly.js');
let Waypoint = require('./Waypoint.js');

module.exports = {
  MissionWaypoint: MissionWaypoint,
  MissionHotpointTask: MissionHotpointTask,
  WaypointList: WaypointList,
  PayloadData: PayloadData,
  MobileData: MobileData,
  FCTimeInUTC: FCTimeInUTC,
  VOPosition: VOPosition,
  GPSUTC: GPSUTC,
  MissionWaypointTask: MissionWaypointTask,
  Gimbal: Gimbal,
  MissionWaypointAction: MissionWaypointAction,
  FlightAnomaly: FlightAnomaly,
  Waypoint: Waypoint,
};
