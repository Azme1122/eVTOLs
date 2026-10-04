function run_derivations
% Derive, export and verify Exercise 8.2 without changing eVTOL.slx.
folder=fileparts(mfilename('fullpath')); addpath(folder);
assert(license('test','Symbolic_Toolbox'),'Symbolic Math Toolbox is required.');
flat=derive_flatness;
dynamic=derive_dynamic_tracking(flat);
plain=flat.plantOutputs;
extended=dynamic.extended;
% Confirm that replacing the input derivative symbols recovers the extension.
syms u1 du1 d2u1 u2 xb2 ub1 real
syms xb1 positive
assert(all(isAlways(simplify(subs(plain.Z1,[u1 du1 d2u1],[xb1 xb2 ub1])-extended.Z1)==0)));
assert(all(isAlways(simplify(subs(plain.Z2,[u1 du1 d2u1],[xb1 xb2 ub1])-extended.Z2)==0)));
% Confirm both routes to dynamic inversion produce identical expressions.
syms v1 v2 real
stateMapping=[flat.x;flat.u1;flat.du1];
stateBased=simplify(subs(extended.control,extended.state,stateMapping.'));
assert(all(isAlways(simplify(stateBased-dynamic.tracking)==0)));
generated=fullfile(folder,'generated');if ~isfolder(generated),mkdir(generated);end
matlabFunction(flat.u1,flat.u2,flat.x(5),flat.x(6), ...
    'File',fullfile(generated,'vtol_feedforward_generated'), ...
    'Vars',{flat.reference,flat.g},'Outputs',{'u1','u2','theta','omega'});
matlabFunction(dynamic.tracking,'File',fullfile(generated,'vtol_dynamic_generated'), ...
    'Vars',{flat.reference,dynamic.v,flat.g},'Outputs',{'extendedInput'});
addpath(generated);
project=fileparts(folder);
addpath(fullfile(project,'simulink_sources','feedforward_style'));
maximumDifference=0;
for mode=1:2
    names={'Point','Circle'};
    dataset=fullfile(project,'results',[names{mode} '_mil_sil.mat']);
    assert(isfile(dataset),'Run regression from the project root before run_derivations.');
    saved=load(dataset,'mil');
    rows=saved.mil.reference;
    for index=1:200:size(rows,1)
        ref=rows(index,:).';
        [a,b]=vtol_feedforward_generated(ref,1);
        [c,d]=feedforward_control(ref(1:5),ref(6:10),struct('g',1));
        maximumDifference=max(maximumDifference,max(abs([a-c;b-d])));
    end
end
assert(maximumDifference<1e-10,'Derived formulas differ from Simulink feedforward source.');
save(fullfile(folder,'symbolic_results.mat'),'plain','flat','extended','dynamic','maximumDifference');
fprintf('PASS symbolic identities, dynamic inversion and feedforward comparison (%.3g)\n',maximumDifference);
end
