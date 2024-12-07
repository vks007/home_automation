
--MariaDB queries
SELECT states_meta.entity_id,COUNT(*) * 100 / (SELECT COUNT(*) FROM states) AS cnt_pct,COUNT(*) AS cnt FROM states
INNER JOIN states_meta ON states_meta.metadata_id = states.metadata_id
GROUP BY states.metadata_id ORDER BY cnt DESC;






--MariaDB queries

