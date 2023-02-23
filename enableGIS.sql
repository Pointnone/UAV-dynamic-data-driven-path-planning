-- Taken from: https://postgis.net/install/
-- Enable PostGIS (as of 3.0 contains just geometry/geography)
 CREATE EXTENSION postgis;
-- enable raster support (for 3+)
CREATE EXTENSION postgis_raster;
-- Enable Topology
CREATE EXTENSION postgis_topology;
-- Enable PostGIS Advanced 3D
-- and other geoprocessing algorithms
-- sfcgal not available with all distributions
CREATE EXTENSION postgis_sfcgal;
-- fuzzy matching needed for Tiger
CREATE EXTENSION fuzzystrmatch;
-- rule based standardizer
CREATE EXTENSION address_standardizer;
-- example rule data set
CREATE EXTENSION address_standardizer_data_us;
-- Enable US Tiger Geocoder
CREATE EXTENSION postgis_tiger_geocoder;

CREATE TABLE dmi (
    geom GEOMETRY,
    precip_meas FLOAT,
    winddir_meas FLOAT,
    windspd_meas FLOAT,
    dt TIMESTAMP,
    sim BOOLEAN,
    sim_layer INTEGER
);

CREATE TABLE df (
    geom GEOMETRY,
    itype VARCHAR(10),
    sim BOOLEAN,
    sim_layer INTEGER
);

CREATE TABLE cell (
    geom GEOMETRY,
    technology VARCHAR(15),
    sim BOOLEAN,
    sim_layer INTEGER
)