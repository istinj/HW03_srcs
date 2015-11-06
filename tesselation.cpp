#include "tesselation.h"

// make normals for each face - duplicates all vertex data
void facet_normals(Mesh* mesh) {
    // allocates new arrays
    auto pos = vector<vec3f>();
    auto norm = vector<vec3f>();
    auto texcoord = vector<vec2f>();
    auto triangle = vector<vec3i>();
    auto quad = vector<vec4i>();
    // froeach triangle
    for(auto f : mesh->triangle) {
        // grab current pos size
        auto nv = (int)pos.size();
        // compute face face normal
        auto fn = normalize(cross(mesh->pos[f.y]-mesh->pos[f.x], mesh->pos[f.z]-mesh->pos[f.x]));
        // add triangle
        triangle.push_back({nv,nv+1,nv+2});
        // add vertex data
        for(auto i : range(3)) {
            pos.push_back(mesh->pos[f[i]]);
            norm.push_back(fn);
            if(! mesh->texcoord.empty()) texcoord.push_back(mesh->texcoord[f[i]]);
        }
    }
    // froeach quad
    for(auto f : mesh->quad) {
        // grab current pos size
        auto nv = (int)pos.size();
        // compute face normal
        auto fn = normalize(normalize(cross(mesh->pos[f.y]-mesh->pos[f.x], mesh->pos[f.z]-mesh->pos[f.x])) +
                            normalize(cross(mesh->pos[f.z]-mesh->pos[f.x], mesh->pos[f.w]-mesh->pos[f.x])));
        // add quad
        quad.push_back({nv,nv+1,nv+2,nv+3});
        // add vertex data
        for(auto i : range(4)) {
            pos.push_back(mesh->pos[f[i]]);
            norm.push_back(fn);
            if(! mesh->texcoord.empty()) texcoord.push_back(mesh->texcoord[f[i]]);
        }
    }
    // set back mesh data
    mesh->pos = pos;
    mesh->norm = norm;
    mesh->texcoord = texcoord;
    mesh->triangle = triangle;
    mesh->quad = quad;
}

// smooth out normal - does not duplicate data
void smooth_normals(Mesh* mesh) {
    // YOUR CODE GOES HERE ---------------------
    // set normals array to the same length as pos and init all elements to zero
	// Array of vec3f, all entries vectors are zero3f;
	auto norm = std::vector<vec3f>(mesh->pos.size(), zero3f);
    // foreach triangle
	for (auto triangle_ : mesh->triangle)
	{
		// compute face normal
		auto cross_prod = cross((mesh->pos[triangle_.y] - mesh->pos[triangle_.x]), 
			(mesh->pos[triangle_.z] - mesh->pos[triangle_.x]));
		auto temp_norm = normalize(cross_prod);
		// accumulate face normal to the vertex normals of each face index
		norm[triangle_.x] += temp_norm;
		norm[triangle_.y] += temp_norm;
		norm[triangle_.z] += temp_norm;
	}
	// foreach quad
	for (auto quad_ : mesh->quad)
	{
		// compute face normal
		auto c1 = cross((mesh->pos[quad_.y] - mesh->pos[quad_.x]),
			(mesh->pos[quad_.z] - mesh->pos[quad_.x]));
		auto c2 = cross((mesh->pos[quad_.y] - mesh->pos[quad_.w]), 
			(mesh->pos[quad_.z] - mesh->pos[quad_.w]));

		auto temp_quad_norm1 = normalize(c1);
		auto temp_quad_norm2 = normalize(c2);
		// accumulate face normal to the vertex normals of each face index
		auto temp_norm = normalize((temp_quad_norm1 + temp_quad_norm2) / 2);
		norm[quad_.x] += temp_norm;
		norm[quad_.y] += temp_norm;
		norm[quad_.z] += temp_norm;
		norm[quad_.w] += temp_norm;
	}
    // normalize all vertex normals
	for (vec3f n_ : norm)
		n_ = normalize(n_);
}

// smooth out tangents
void smooth_tangents(Mesh* polyline) {
    // set tangent array
    polyline->norm = vector<vec3f>(polyline->pos.size(),zero3f);
    // foreach line
    for(auto l : polyline->line) {
        // compute line tangent
        auto lt = normalize(polyline->pos[l.y]-polyline->pos[l.x]);
        // accumulate segment tangent to vertex tangent on each vertex
        for (auto i : range(2)) polyline->norm[l[i]] += lt;
    }
    // normalize all vertex tangents
    for (auto& t : polyline->norm) t = normalize(t);
}

// apply Catmull-Clark mesh subdivision
// does not subdivide texcoord
void subdivide_catmullclark(Mesh* subdiv) {
    // YOUR CODE GOES HERE ---------------------
    // skip is needed
	if (subdiv->subdivision_catmullclark_level == 0) return;
    
	// allocate a working Mesh copied from the subdiv
	Mesh* temp_mesh = subdiv;
    // foreach level
	for (int i = 0; i < temp_mesh->subdivision_catmullclark_level; i++)
	{
		// make empty pos and quad arrays
		auto pos_array = std::vector<vec3f>();
		auto quad_array = std::vector<vec4i>(temp_mesh->triangle.size() * 3 + temp_mesh->quad.size() * 4, zero4i);

		// create edge_map from current mesh
		EdgeMap edgeM_ = EdgeMap(temp_mesh->triangle, temp_mesh->quad);
		
		// linear subdivision - create vertices
		// copy all vertices from the current mesh
		pos_array = temp_mesh->pos;

		// add vertices in the middle of each edge (use EdgeMap)
		vec3f midpoint;
		auto e_list = edgeM_.edges();
		auto midpoint_array = std::vector<vec3f>(e_list.size(), zero3f);
		for (auto ed_ : e_list)
		{
			int startIndex = ed_.x;
			int finalInedx = ed_.y;
			midpoint = (temp_mesh->pos[startIndex] + temp_mesh->pos[finalInedx])/2;
			midpoint_array[edgeM_.edge_index(ed_)] = midpoint; 
			//edgeM_._add_edge(startIndex, finalInedx);
		}

		// add vertices in the middle of each triangle
		vec3f tri_centroid;
		auto tri_centroid_array = std::vector<vec3f>(temp_mesh->triangle.size(), zero3f);
		int k = 0;
		for (auto triangle_ : temp_mesh->triangle)
		{
			tri_centroid = (temp_mesh->pos[triangle_.x] + 
				temp_mesh->pos[triangle_.y] + 
				temp_mesh->pos[triangle_.z])/3;
			tri_centroid_array[k] = tri_centroid;
			k++;
		}

		// add vertices in the middle of each quad
		vec3f quad_centroid;
		auto quad_centroid_array = std::vector<vec3f>(temp_mesh->quad.size(), zero3f);
		k = 0;
		for (auto quad_ : temp_mesh->quad)
		{
			quad_centroid = (temp_mesh->pos[quad_.x] + 
				temp_mesh->pos[quad_.y] + 
				temp_mesh->pos[quad_.z] + 
				temp_mesh->pos[quad_.w])/4;
			quad_centroid_array[k] = quad_centroid;
			k++;
		}

		// Append all the vertices in pos-array vector
		pos_array.insert(pos_array.end(), midpoint_array.begin(), midpoint_array.end());
		pos_array.insert(pos_array.end(), tri_centroid_array.begin(), tri_centroid_array.end());
		pos_array.insert(pos_array.end(), quad_centroid_array.begin(), quad_centroid_array.end());

		// subdivision pass --------------------------------
		// compute an offset for the edge vertices
		auto edge_offset = temp_mesh->pos.size();
		// compute an offset for the triangle vertices
		auto tri_offset = edge_offset + midpoint_array.size();
		// compute an offset for the quad vertices
		auto quad_offset = tri_offset + tri_centroid_array.size();

		// foreach triangle
		for (int i = 0; i < temp_mesh->triangle.size(); i+=3)
		{
			quad_array[i] = vec4i(	i,
									edge_offset + i, 
									tri_offset + i, 
									edge_offset + 1 + i);

			quad_array[i + 1] = vec4i(	(i + 1), 
										edge_offset + 1 + (i + 1), 
										tri_offset + i, 
										edge_offset + i);

			quad_array[i + 2] = vec4i(	(i + 2), 
										edge_offset + (i + 1), 
										tri_offset + i, 
										edge_offset + 1 + (i + 1));
		}
		
		// foreach quad
		//midpoint_offset = edge_offset e qua
		for ( ; i < temp_mesh->triangle.size() * 3 + temp_mesh->quad.size(); i += 4)
		{
			// add four quads to the new quad array
			quad_array[i] = vec4i(	i,
									edge_offset + i,
									quad_offset + i,
									edge_offset + i + 3);

			quad_array[i] = vec4i(	i + 1,
									edge_offset + i + 1,
									quad_offset + i,
									edge_offset + i);

			quad_array[i] = vec4i(	i + 2,
									edge_offset + i + 2,
									quad_offset + i,
									edge_offset + i + 1);

			quad_array[i] = vec4i(	i + 3,
									edge_offset + i + 3,
									quad_offset + i,
									edge_offset + i + 2);
		}

		// averaging pass ----------------------------------
		// create arrays to compute pos averages (avg_pos, avg_count)
		// arrays have the same length as the new pos array, and are init to zero
		auto avg_pos = std::vector<vec3f>(pos_array.size(), zero3f);
		auto avg_count = std::vector<int>(pos_array.size(), 0);
		// for each new quad
		for (auto quad_ : quad_array)
		{
			// compute quad center using the new pos array
			auto c = (	pos_array[quad_.x] +
						pos_array[quad_.y] +
						pos_array[quad_.z] +
						pos_array[quad_.w]) / 4;

			// foreach vertex index in the quad
			for (int j = 0; j < 4; j++)
			{
				int v_index = quad_[j];
				avg_pos[v_index] += c;
				avg_count[v_index]++;
			}
		}

		// normalize avg_pos with its count avg_count
		for (int i = 0; i < avg_pos.size(); i++)
			avg_pos[i] = avg_pos[i] / avg_count[i];

		// correction pass ----------------------------------
		// foreach pos, compute correction p = p + (avg_p - p) * (4/avg_count)
		for (int v_index = 0; v_index < avg_pos.size(); v_index++)
			pos_array[v_index] += (avg_pos[v_index] - pos_array[v_index]) * (4 / avg_count[v_index]);

		// set new arrays pos, quad back into the working mesh; clear triangle array
		temp_mesh->pos = pos_array;
		temp_mesh->quad = quad_array;
		temp_mesh->triangle.clear();
		//delete[] &temp_mesh->triangle;
	}
    // clear subdivision
	subdiv->pos.clear();
	subdiv->triangle.clear();
	subdiv->quad.clear();

    // according to smooth, either smooth_normals or facet_normals
	facet_normals(temp_mesh);

    // copy back
	subdiv = temp_mesh;

    // clear
	delete[] temp_mesh;
}

// subdivide bezier spline into line segments (assume bezier has only bezier segments and no lines)
void subdivide_bezier(Mesh* bezier) {
    // YOUR CODE GOES HERE ---------------------
    // skip is needed
	if (bezier->subdivision_bezier_level == 0) return;
    // allocate a working polyline from bezier
	Mesh* w_poly = new Mesh(*bezier);
    // foreach level
	for (int i = 0; i < w_poly->subdivision_bezier_level; i++)
	{
		// make new arrays of positions and bezier segments
		// copy all the vertices into the new array (this waste space but it is easier for now)
		// foreach bezier segment
		// apply subdivision algorithm
		// prepare indices for two new segments
		// add mid point
		// add points for first segment and fix segment indices
		// add points for second segment and fix segment indices
		// add indices for both segments into new segments array
		// set new arrays pos, segments into the working lineset
	}

    // copy bezier segments into line segments
    // clear bezier array from lines
    // run smoothing to get proper tangents
    // copy back
    // clear
}

Mesh* make_surface_mesh(frame3f frame, float radius, bool isquad, Material* mat, float offset) {
    auto mesh = new Mesh{};
    mesh->frame = frame;
    mesh->mat = mat;
    if(isquad) {
        mesh->pos = { {-radius,-radius,-offset}, {radius,-radius,-offset},
            {radius,radius,-offset}, {-radius,radius,-offset} };
        mesh->norm = {z3f,z3f,z3f,z3f};
        mesh->quad = { {0,1,2,3} };
    } else {
        map<pair<int,int>,int> vid;
        for(auto j : range(64+1)) {
            for(auto i : range(128+1)) {
                auto u = 2 * pif * i / 64.0f, v = pif * j / 32.0f;
                auto d = vec3f{cos(u)*sin(v),sin(u)*sin(v),cos(v)};
                vid[{i,j}] = mesh->pos.size();
                mesh->pos.push_back(d*radius*(1-offset));
                mesh->norm.push_back(d);
            }
        }
        for(auto j : range(64)) {
            for(auto i : range(128)) {
                mesh->quad.push_back({vid[{i,j}],vid[{i+1,j}],vid[{i+1,j+1}],vid[{i,j+1}]});
            }
        }
    }
    return mesh;
}

void subdivide_surface(Surface* surface) {
    surface->_display_mesh = make_surface_mesh(
        surface->frame, surface->radius, surface->isquad, surface->mat);
}

void subdivide(Scene* scene) {
    for(auto mesh : scene->meshes) {
        if(mesh->subdivision_catmullclark_level) subdivide_catmullclark(mesh);
        if(mesh->subdivision_bezier_level) subdivide_bezier(mesh);
    }
    for(auto surface : scene->surfaces) {
        subdivide_surface(surface);
    }
}
