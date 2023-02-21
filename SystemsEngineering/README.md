# Product Development & Systems Engineering

## Section 1. Introductory / Groundwork Topics

### 1. Course Introduction
<img src="images/intro.png"  width="600" height="300">

* Everything, including complex products and systems, has a life cycle.
* Systems engineering is an approach to bringing complex products into existence from the top down, starting with defining characteristics at high levels and then breaking them down into smaller and smaller segments until implementation.
* Successful systems are developed using a win-win strategy where the customer and supplier are both better off and happier after the transaction.
* Many systems engineering courses are either too detailed or too short, too specific, or not offered online.
* This course was designed to fill the gap in the learning market by focusing on the big picture of product development and systems engineering.
* This course is primarily aimed at teams that want to bring complex products into existence, grad school students, research labs, and startup companies seeking to develop and field their inventions.
* This course is beneficial for large firms specializing in developing or improving already existing complex systems, including jetliner manufacturers, and government organizations that define and oversee system development.
* The course aligns with the Inkosi Systems Engineering Handbook and ISO IEC II Tripoli Standard 15 to 88, and helps reduce the risk of overspending budget or exceeding schedule.
* It teaches how to plan and organize products by architecting the structure of the system from the top down, identifying system requirements, interactions, internal functions, and components, estimating the scope of development efforts, and identifying required infrastructure.
* The course teaches how to successfully define, develop, implement, integrate, test, produce, deploy, and sustain a product while increasing reliability, improving quality, and reducing overall risk.
* The course uses examples of aircraft, cars, and automated teller machines, with the latter being the most used example.
* The course focuses on systems engineering and uses the ATM as the primary example system.
* The first set of lessons covers the definition of systems, systems engineering, benefits, systems context and boundaries, and systems engineering activities.
* The second set of lessons covers each of the life cycle technical processes, including writing requirements, creating functions and functional flows, writing use cases, planning for tests, and constructing the physical characteristics of the system.
* The third set of lessons covers the technical management processes, such as risk management, configuration management, project planning, project management, and quality management.
* The fourth set of lessons covers the agreement processes, including the roles of the developer and government in the acquisition and supply processes.
* The fifth set of lessons covers the enterprise processes or project enabling processes, including lifecycle management, infrastructure management, portfolio and human resource management, and portfolio and knowledge management.
* The sixth set of lessons covers tailoring the processes, modeling and simulation, object-oriented systems engineering, software development prototyping, integrated product development, lean systems engineering, agile systems engineering, and enterprise systems.
* The seventh set of lessons covers subject matter areas such as affordability and value engineering, electromagnetic compatibility, environmental engineering, reliability system safety, human system integration, system security, product support, and resilience.
* The instructor, Christopher Olsen, has extensive experience in product development and systems engineering, including working on various programs for the US Army, civil aviation, and the US Navy.
* The instructor is also certified in DOWA, PMP, and OC, SMP.
* The course has over 100 lessons, each averaging about 15 minutes, and is designed to help learners develop a product.

### 2. System Defined

<img src="images/system_defined.png" width="600" height="300">
<img src="images/integrated.png" width="600" height="300">

<img src="images/elements.png" width="600" height="300">
<img src="images/definitions.png" width="600" height="300">

* The course focuses on the concept of the system, which is defined as a combination of interacting elements organized to achieve one or more stated purposes.
* Interacting elements refer to a group of parts within the system that work together, such as hardware, software, processes, people, information, techniques, and facilities.
* System elements provide internal and external services, such as those for the user, and are critical to product development.
* The concept of services will come up frequently throughout the course.
* The definition of a system is a combination of interacting elements organized to achieve one or more stated purposes, with stated purposes being the agreed-upon primary goal of the system.
* Other sources provide similar definitions, indicating that a system is a human-made entity created to execute a mission for a user with a specific operational environment.
* A desktop computer is an example of a system, with its various integrated elements working together to achieve defined objectives.
* The course will use the ISO definition of a system throughout.

### 3. System Hierarchies

<img src="images/systems_subsystems.png" width="600" height="300">

<img src="images/subsystems_components.png" width="600" height="300">

<img src="images/components_subcomponents.png" width="600" height="300">

<img src="images/hierarchy.png" width="600" height="300">

<img src="images/uml_diagram.png" width="600" height="300">

<img src="images/automobile_hierarchy.png" width="600" height="300">

<img src="images/context_dependent.png" width="600" height="300">

<img src="images/power_subsystem_hierarchy.png" width="600" height="300">

<img src="images/domain_specific.png" width="600" height="300">

<img src="images/how_far_to_decompose.png" width="600" height="300">

<img src="images/hierarchies_are_result.png" width="600" height="300">

#### Introduction to System Hierarchy

* Creating a system hierarchy helps to decompose a system into smaller, manageable pieces.
* A system hierarchy is an organizational representation of a system structure using partitioning relationships.

#### Components of the Hierarchy

The hierarchy is composed of the system at the top, followed by subsystems, components, subcomponents, and parts.
* The subsystem layer is a collection of system elements that fulfill the primary goals of the overall system.
* Components work together to perform the operations of a subsystem, while subcomponents operate together to allow each component to perform its primary functions.
* Parts are individual items that make up the subcomponents.

#### Expressing the Hierarchy

* A system's hierarchy is usually expressed using words rather than graphics.
* Composite associations are shown as a little black diamond, with "is comprised of" used when reading from top to bottom, and "is a part of" when reading from bottom to top.
* The relationships are typically expressed using words to describe system elements or using graphics like the small block definition diagram.
* The diagram shows the composition relationships and multiplicity of system elements.

#### Context and Use

* The hierarchy depends on the context of use, and it may be expressed differently by different organizations or manufacturers.
* System hierarchies are dependent on context and specific to various domains.
* It is important to decompose the hierarchy down to a level where the individual element in question can be developed and integrated with other elements.
* Multiplicity can be used to show that a subcomponent is composed of more than one part of the same configuration.

#### Conclusion

* Systems engineers use system hierarchy to break down a system into smaller, more manageable parts.
* System hierarchy is an organizational representation of a system structure using partitioning relationships.
* Hierarchies are a product of developing a system, not created first and then followed up by the design of the system.

### 4. The System Context

<img src="images/external_view.png" width="600" height="300">

<img src="images/system_environment.png" width="600" height="300">

<img src="images/external_entities.png" width="600" height="300">

<img src="images/jet_fighter_example.png" width="600" height="300">

<img src="images/system_boundaries.png" width="600" height="300">

<img src="images/system_boundaries_jetfighter.png" width="600" height="300">

<img src="images/in_or_out.png" width="600" height="300">

<img src="images/user_fit.png" width="600" height="300">

<img src="images/context_diagram.png" width="600" height="300">

<img src="images/context_diagram_jetfighter.png" width="600" height="300">

<img src="images/context_diagram_format_2.png" width="600" height="300">

<img src="images/internal_block_diagram.png" width="600" height="300">

<img src="images/ports.png" width="600" height="300">

<img src="images/vehicle_engine_context.png" width="600" height="300">

<img src="images/how_to_build_context_diagram.png" width="600" height="300">

<img src="images/final_points.png" width="600" height="300">

### 5. System Domains

<img src="images/systems_eng_knowledge_breadth.png" width="600" height="300">

<img src="images/compartmentalizing.png" width="600" height="300">

<img src="images/requirement_mission_function_entity.png" width="600" height="300">

<img src="images/requirements.png" width="600" height="300">

<img src="images/requirements_example_nasa.png" width="600" height="300">

<img src="images/system_operations.png" width="600" height="300">

<img src="images/use_case_example.png" width="600" height="300">

<img src="images/functions.png" width="600" height="300">

<img src="images/functional_flow_diagram_example.png" width="600" height="300">

<img src="images/functional_flow_diagram_zoom_in.png" width="600" height="300">

<img src="images/physical_elements.png" width="600" height="300">

<img src="images/product_hierarchy.png" width="600" height="300">

### 6. Users & Stakeholders

<img src="images/success_of_a_product.png" width="600" height="300">

<img src="images/user.png" width="600" height="300">

<img src="images/user_types.png" width="600" height="300">

<img src="images/user_atm_example.png" width="600" height="300">

<img src="images/6_stakeholder.png" width="600" height="300">

<img src="images/6_user_vs_stake_holder.png" width="600" height="300">

<img src="images/6_role_of_users_and_stakeholders.png" width="600" height="300">

### 7. Boundaries & Interfaces

<img src="images/7_boundaries_atm_example.png" width="600" height="300">

<img src="images/7_interfaces.png" width="600" height="300">

<img src="images/7_interface_types.png" width="600" height="300">

<img src="images/7_connector_interface.png" width="600" height="300">

<img src="images/7_isolator_interface.png" width="600" height="300">

<img src="images/7_converter_interface.png" width="600" height="300">

<img src="images/7_context_diagram.png" width="600" height="300">

<img src="images/7_engine_example.png" width="600" height="300">

<img src="images/7_engine_controller_engine.png" width="600" height="300">

<img src="images/7_interface_engine_example.png" width="600" height="300">

<img src="images/7_standardized_interfaces_1.png" width="600" height="300">

<img src="images/7_standardized_interfaces_2.png" width="600" height="300">

<img src="images/7_interface_management.png" width="600" height="300">

<img src="images/7_evolution_of_usb_interface.png" width="600" height="300">

### 8. Systems Engineering Overview

<img src="8_systems_engineering.png" width="600" height="300">

<img src="8_systems_engineering_2.png" width="600" height="300">

<img src="8_systems_engineering_process.png" width="600" height="300">

<img src="8_systems_engineering_process.png" width="600" height="300">

<img src="8_system_hierarchy.png" width="600" height="300">

<img src="8_activity_decomposition.png" width="600" height="300">

<img src="8_system_hierarchy_2.png" width="600" height="300">

<img src="8_analysis_and_control.png" width="600" height="300">

<img src="8_system_engineering_iteration.png" width="600" height="300">

<img src="8_system_engineering_iteration_2.png" width="600" height="300">

<img src="8_system_engineering_recursion.png" width="600" height="300">
