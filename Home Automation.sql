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


    value_template: >-
      {% if is_state('value_json.SR04.Distance', 'unknown') %}
      0.0
      {% else %}
      value_json.SR04.Distance
      {% endif %}


  - platform: filter
    name: "Filtered water level lowpass"
    entity_id: sensor.overhead_tank
    filters:
      - filter: lowpass
        time_constant: 10

  - platform: filter
    name: "Filtered water level tsma lowpass"
    entity_id: sensor.overhead_tank
    filters:
      - filter: time_simple_moving_average
        window_size: 01:00
        precision: 1
      - filter: lowpass
        time_constant: 10

    - filter: time_simple_moving_average
      window_size: 1:00
      precision: 0
    - filter: lowpass
      time_constant: 10
	  


DELETE 
FROM "states" WHERE entity_id like "sensor.filtered_water_level%" 

SELECT entity_id,created,state
FROM "states" WHERE entity_id like "sensor.%water_level%"  ORDER BY state_id DESC LIMIT 50
 
