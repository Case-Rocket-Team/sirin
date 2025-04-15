use anyhow::bail;
use proc_macro::TokenStream;
#[allow(unused_imports)]
use quote::ToTokens;
use syn::{Attribute, DataEnum, DataStruct, Field, Ident, Meta, MetaList, Token};
#[allow(unused_imports)]
use syn::{parse::Parser, parse_macro_input, DeriveInput};
use quote::quote;
use proc_macro2::{Delimiter, TokenStream as TokenStream2, TokenTree};

#[proc_macro_derive(Serialize, attributes(song))]
pub fn derive_serialize(item: TokenStream) -> TokenStream {
    let item: DeriveInput = parse_macro_input!(item);
    let ident = item.ident;

    match item.data {
        syn::Data::Struct(s) => derive_serialize_struct(ident, s).unwrap().into(),
        syn::Data::Enum(e) => derive_serialize_enum(ident, e, item.attrs).unwrap().into(),
        _ => todo!()
    }
}

// TODO UNFINISHED
fn derive_serialize_enum(ident: Ident, item: DataEnum, attrs: Vec<Attribute>) -> Result<TokenStream2, anyhow::Error> {
    let attrs: Vec<TokenStream2> = attrs.iter().filter_map(|a| {
        if a.path().get_ident()?.to_string() != "song" {
            return None;
        }

        let Meta::List(ref list) = a.meta else {
            return None
        };

        Some(list.tokens.clone())
    }).collect();

    let mut desc_ty_name = None;
    let mut desc_ty = None;

    for attr in attrs {
        if desc_ty_name.is_some() {
            bail!("Only one descriminant allowed");
        }

        let tokens: Vec<TokenTree> = attr.into_iter().collect();

        let Some(ident) = tokens.get(0) else {
            bail!("Expected an ident");
        };

        let tokens: Vec<TokenTree> = group.stream().into_iter().collect();

        if tokens.len() != 3 {
            bail!("Should be in the form #[song(descriminant = DescriminantType)]");
        }

        let toks = (tokens[0].clone(), tokens[1].clone(), tokens[2].clone());

        let (
            TokenTree::Ident(ty_name),
            TokenTree::Punct(punct),
            TokenTree::Ident(ty)
        ) = toks else {
            bail!("Should be in the form #[song(descriminant(DescriminantType = u8))]");
        };

        if punct.as_char() != '=' {
            bail!("Should be in the form #[song(descriminant(DescriminantType = u8))]");
        }

        desc_ty_name = Some(ty_name);
        desc_ty = Some(ty);
    }

    let tmp = (desc_ty_name, desc_ty);

    let (Some(desc_ty_name), Some(desc_ty)) = tmp else {
        bail!("Descriminant type is required for enum");
    };

    desc_ty_name
}

fn derive_serialize_struct(ident: Ident, item: DataStruct) -> Result<TokenStream2, anyhow::Error> {
    let mut serialize_out = vec![];
    let mut size_out = vec![];

    for field in item.fields {
        let Field { ident, .. } = field;

        size_out.push(quote! {
            + self.#ident.serialization_size()
        });

        serialize_out.push(quote! {
            self.#ident.serialize(buf[i..])?;
            i += self.#ident.serialization_size();
        });
    }

    Ok(quote! {
        impl SerializationSize for #ident {
            fn serialization_size(&self) -> usize {
                0 + #(#size_out)*
            }
        }

        impl Serialize for #ident {
            fn serialize(&self, buf: &mut [u8]) -> Result<(), SerializationError> {
                let size = self.serialization_size();
                if buf.len() >= size {
                    let mut i = 0;
                    #(#serialize_out)*
                    Ok(())
                } else {
                    Err(SerializationError::NotEnoughBytes)
                }
            }
        }
    })
}