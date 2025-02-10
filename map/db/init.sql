CREATE TABLE IF NOT EXISTS spatial_point (
    x INTEGER,
    y INTEGER,
    z INTEGER,
    color_r SMALLINT,
    color_g SMALLINT,
    color_b SMALLINT,
    color_a SMALLINT,
    timestamp FLOAT,
    nb_records INTEGER,
    PRIMARY KEY (x, y, z)
);

CREATE OR REPLACE FUNCTION spatial_point_upsert()
RETURNS TRIGGER AS $$
BEGIN
  -- Vérifier si une ligne avec la même clé primaire existe déjà
  IF EXISTS (SELECT 1 FROM spatial_point WHERE x = NEW.x AND y = NEW.y AND z = NEW.z) THEN
    -- Mettre à jour la ligne existante avec les nouvelles valeurs
    UPDATE spatial_point
    SET color_r = NEW.color_r,
        color_g = NEW.color_g,
        color_b = NEW.color_b,
        color_a = NEW.color_a,
        timestamp = NEW.timestamp,
        nb_records = nb_records + 1
    WHERE x = NEW.x AND y = NEW.y AND z = NEW.z;
    RETURN NULL; -- Ignorer l'opération d'insertion en cours
  ELSE
    -- Pas de conflit de clé primaire, effectuer l'insertion
    RETURN NEW;
  END IF;
END;
$$ LANGUAGE plpgsql;

-- Créer le trigger sur la table spatial_point
CREATE OR REPLACE TRIGGER spatial_point_upsert_trigger
BEFORE INSERT ON spatial_point
FOR EACH ROW
EXECUTE FUNCTION spatial_point_upsert();