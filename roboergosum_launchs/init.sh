rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: false, toasterSimuHuman: true, pr2Robot: true, spencerRobot: false, toasterSimuRobot: false, toasterSimuObject: true}"

rosservice call /database_manager/execute "{command: 'EMPTY', type: 'ALL', facts: [], agent: '', order: '', areaTopic: false ,agentTopic: false ,move3dTopic: false ,pdgTopic: false}"

rosservice call /toaster_simu/add_entity "{id: 'TABLE_4', name: 'TABLE_4', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'TABLE_4', ownerId: '', type: 'object', pose:
{position:
  {x: 4.6,
  y: 4.0,
  z: 0.0},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'GREEN_TRASHBIN', name: 'GREEN_TRASHBIN', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'GREEN_TRASHBIN', ownerId: '', type: 'object', pose:
{position:
  {x: 4.1,
  y: 3.1,
  z: 0.751},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'BLUE_TRASHBIN', name: 'BLUE_TRASHBIN', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'BLUE_TRASHBIN', ownerId: '', type: 'object', pose:
{position:
  {x: 5.1,
  y: 5.1,
  z: 0.751},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'PLACEMAT_1', name: 'PLACEMAT_1', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'PLACEMAT_1', ownerId: '', type: 'object', pose:
{position:
  {x: 4.4,
  y: 3.6,
  z: 0.72},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'PLACEMAT_2', name: 'PLACEMAT_2', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'PLACEMAT_2', ownerId: '', type: 'object', pose:
{position:
  {x: 4.35,
  y: 3.4,
  z: 0.72},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'PLACEMAT_3', name: 'PLACEMAT_3', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'PLACEMAT_3', ownerId: '', type: 'object', pose:
{position:
  {x: 4.6,
  y: 3.7,
  z: 0.72},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'PLACEMAT_4', name: 'PLACEMAT_4', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'PLACEMAT_4', ownerId: '', type: 'object', pose:
{position:
  {x: 4.6,
  y: 4.3,
  z: 0.751},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'PLACEMAT_5', name: 'PLACEMAT_5', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'PLACEMAT_5', ownerId: '', type: 'object', pose:
{position:
  {x: 4.75,
  y: 4.3,
  z: 0.72},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.7,
  w: 0.7}}}"

rosservice call /toaster_simu/add_entity "{id: 'PLACEMAT_6', name: 'PLACEMAT_6', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'PLACEMAT_6', ownerId: '', type: 'object', pose:
{position:
  {x: 4.8,
  y: 4.5,
  z: 0.72},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'PLACEMAT_7', name: 'PLACEMAT_7', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'PLACEMAT_7', ownerId: '', type: 'object', pose:
{position:
  {x: 4.9,
  y: 4.6,
  z: 0.72},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 0.0,
  w: 1.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'GREEN_TAPE1', name: 'GREEN_TAPE1', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/add_entity "{id: 'GREEN_TAPE2', name: 'GREEN_TAPE2', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/add_entity "{id: 'GREEN_TAPE3', name: 'GREEN_TAPE3', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/add_entity "{id: 'BLUE_TAPE1', name: 'BLUE_TAPE1', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/add_entity "{id: 'BLUE_TAPE2', name: 'BLUE_TAPE2', type: 'object', ownerId: ''}"
rosservice call /toaster_simu/add_entity "{id: 'BLUE_TAPE3', name: 'BLUE_TAPE3', type: 'object', ownerId: ''}"



rosservice call /toaster_simu/add_entity "{id: 'HERAKLES_HUMAN1', name: 'HERAKLES_HUMAN1', type: 'human', ownerId: ''}"
rosservice call /toaster_simu/set_entity_pose "{id: 'HERAKLES_HUMAN1', ownerId: '', type: 'human', pose:
{position:
  {x: 5.5,
  y: 4.2,
  z: 0.0},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 1.0,
  w: 0.0}}}"


rosservice call /toaster_simu/add_entity "{id: 'rightHand', name: 'rightHand', type: 'joint', ownerId: 'HERAKLES_HUMAN1'}"
rosservice call /toaster_simu/set_entity_pose "{id: 'rightHand', ownerId: 'HERAKLES_HUMAN1', type: 'joint', pose:
{position:
  {x: 5.6,
  y: 4.55,
  z: 1.0},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 1.0,
  w: 0.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'base', name: 'base', type: 'joint', ownerId: 'HERAKLES_HUMAN1'}"
rosservice call /toaster_simu/set_entity_pose "{id: 'base', ownerId: 'HERAKLES_HUMAN1', type: 'joint', pose:
{position:
  {x: 5.5,
  y: 4.2,
  z: 0.0},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 1.0,
  w: 0.0}}}"

rosservice call /toaster_simu/add_entity "{id: 'head', name: 'head', type: 'joint', ownerId: 'HERAKLES_HUMAN1'}"
rosservice call /toaster_simu/set_entity_pose "{id: 'head', ownerId: 'HERAKLES_HUMAN1', type: 'joint', pose:
{position:
  {x: 5.5,
  y: 4.2,
  z: 1.5},
orientation:
  {x: 0.0,
  y: 0.0,
  z: 1.0,
  w: 0.0}}}"

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'robotReachable'
  myOwner: 'TABLE_4'
  areaType: ''
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 2.0
  poly:
    points:
    - {x: -0.6, y: 0.4, z: 0}
    - {x: -0.6, y: -1.0, z: 0}
    - {x: 0.2, y: -1.0, z: 0}
    - {x: 0.2, y: 0.4, z: 0}
    - {x: -0.6, y: 0.4, z: 0}
  insideEntities: [0]"

rosservice call /area_manager/add_area "myArea:
    id: 1
    name: 'humanReachable'
    myOwner: 'TABLE_4'
    areaType: ''
    factType: ''
    entityType: 'entities'
    isCircle: false
    center: {x: 0.0, y: 0.0, z: 0.0}
    ray: 2.0
    poly:
      points:
      - {x: -0.1, y: 0.6, z: 0}
      - {x: -0.1, y: -0.4, z: 0}
      - {x: 1.2, y: -0.4, z: 0}
      - {x: 1.2, y: 0.6, z: 0}
      - {x: -0.1, y: 0.6, z: 0}
    insideEntities: [0]"

rosservice call /area_manager/add_area "myArea:
    id: 2
    name: 'secondReachable'
    myOwner: 'TABLE_4'
    areaType: ''
    factType: ''
    entityType: 'entities'
    isCircle: false
    center: {x: 0.0, y: 0.0, z: 0.0}
    ray: 2.0
    poly:
      points:
      - {x: -0.2, y: 1.2, z: 0}
      - {x: -0.2, y: -0.0, z: 0}
      - {x: 0.6, y: -0.0, z: 0}
      - {x: 0.6, y: 1.2, z: 0}
      - {x: -0.2, y: 1.2, z: 0}
    insideEntities: [0]"

rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: { frame_id: "/map" }, pose: { pose: { position: { x: 4.0, y: 4.2 }, orientation: { x: 0, y: 0, z: 0.0, w: 1.0 } }, covariance: [ 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ] } }'
