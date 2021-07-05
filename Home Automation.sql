
--MariaDB queries
SELECT * FROM `states` WHERE entity_id = "device_tracker.xuv_5945" AND attributes LIKE "%\"ignition\": true%" order by created DESC
SELECT * FROM `states` WHERE entity_id = "sensor.tank_pressure" AND attributes LIKE "%\"ignition\": true%" order by created DESC
SELECT * FROM `states` WHERE entity_id LIKE "%pressure%" AND attributes LIKE "%\"Kpascal\"%" LIMIT 50
UPDATE states SET attributes = "{\"state_class\": \"measurement\", \"unit_of_measurement\": "%", \"friendly_name\": \"Tank Water Pressure\", \"device_class\": \"voltage\"}"
UPDATE homeassistant.states SET attributes = REPLACE(attributes, 'KPascal', '%') WHERE state_id = 3657819
UPDATE homeassistant.states s SET s.attributes = REPLACE (s.attributes, 'KPascal', '%') WHERE s.attributes LIKE '%KPascal%' AND state_id = 3657819;



-- SELECT * FROM `states` WHERE entity_id LIKE "%pressure%" LIMIT 50;
UPDATE homeassistant.states s SET s.attributes = REPLACE (s.attributes, 'Kpascal', '%') WHERE s.attributes LIKE '%Kpascal%' AND entity_id = "sensor.tank_pressure_exp"; 
-- select * from states WHERE state_id = 3657819 ;
DELETE FROM homeassistant.states WHERE entity_id IN ("sensor.tank_water_pressure","sensor.tank_pressure_avg", "sensor.tank_pressure_med","sensor.tank_pressure_exp") AND state < 35


pressure


--MariaDB queries

--My SQL queries

SELECT state_id,created,state
FROM "states" WHERE entity_id = "sensor.overhead_tank" ORDER BY state_id DESC 

DELETE FROM "states" 
WHERE entity_id = "sensor.overhead_tank" 

SELECT entity_id,created,state
FROM "states" WHERE entity_id = "sensor.overhead_tank"  ORDER BY state_id DESC LIMIT 50

SELECT entity_id,created,state
FROM "states" WHERE entity_id = "sensor.overhead_tank" 
AND created > '2019-07-13' ORDER BY state_id DESC

DELETE FROM "states" 
WHERE entity_id = "sensor.overhead_tank" 


2019-06-29 17:48:03.235263 = 11:18 PM


DELETE 
FROM "states" WHERE entity_id like "sensor.filtered_water_level%" 

SELECT entity_id,created,state
FROM "states" WHERE entity_id like "sensor.%water_level%"  ORDER BY state_id DESC LIMIT 50
 
--My SQL queries


{"state_class": "measurement", "unit_of_measurement": "Kpascal", "friendly_name": "Tank Water Pressure", "device_class": "voltage"}
